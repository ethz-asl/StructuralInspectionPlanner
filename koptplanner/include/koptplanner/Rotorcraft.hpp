/*!
 * \file Rotorcraft.hpp
 *
 * More elaborate description
 */
#ifndef __ROTORCRAFT_HPP__
#define __ROTORCRAFT_HPP__

#include "ros/ros.h"
#include "koptplanner/Rotorcraft.h"
#include "koptplanner/plan.hpp"
#include "koptplanner/FixedWing.h"
#include "float.h"
#include <Eigen/Geometry>
#include "optec/qpOASES.hpp"
#include <shape_msgs/Plane.h>
#include <visualization_msgs/Marker.h>
#include "tf/tf.h"
#include <ros/package.h>
#include <fstream>

#define X_MIN (problemBoundary.center[0]-problemBoundary.size[0]/2)
#define Y_MIN (problemBoundary.center[1]-problemBoundary.size[1]/2)
#define Z_MIN (problemBoundary.center[2]-problemBoundary.size[2]/2)
#define X_MAX (problemBoundary.center[0]+problemBoundary.size[0]/2)
#define Y_MAX (problemBoundary.center[1]+problemBoundary.size[1]/2)
#define Z_MAX (problemBoundary.center[2]+problemBoundary.size[2]/2)

#ifdef __TIMING_INFO__
 extern long time_DBS;
 extern long time_RRTS;
 extern long time_RRTS_req;
 extern long time_LKH;
#endif

extern double g_camAngleHorizontal;
extern double g_camAngleVertical;
extern double g_camPitch;
extern MDL_NAMESPACE::region problemBoundary;
extern koptError_t koptError;
extern double g_const_D;
extern int g_max_obstacle_depth;
extern double g_discretization_step;
extern double g_angular_discretization_step;
extern double g_security_distance;

template<class System_t, class State_t, class Vector_t, class region_t>
Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>::Triangle()
{
  this->Fixpoint = false;
  this->VPSolver = new QProblem( 3,8 );
  this->H = new real_t[3*3];
  this->A = new real_t[8*3];
  this->d = new real_t[3];
  this->lbA = new real_t[8];
  this->ubA = new real_t[8];
}

template<class System_t, class State_t, class Vector_t, class region_t>
void Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>::init()
{
  this->a = ((this->x2-this->x1).cross(this->x3-this->x2))/2;

  this->aabs = this->a/this->a.norm();
  Vector3f q1 = this->x2-this->x1; q1.normalize();
  AngleAxisf m1 = AngleAxisf(this->incidenceAngle,q1);
  this->n1 = m1*this->a;
  this->n1.normalize();
  Vector3f q2 = this->x3-this->x2; q2.normalize();
  AngleAxisf m2 = AngleAxisf(this->incidenceAngle,q2);
  this->n2 = m2*this->a;
  Vector3f q3 = this->x1-this->x3; q3.normalize();
  this->n2.normalize();
  AngleAxisf m3 = AngleAxisf(this->incidenceAngle,q3);
  this->n3 = m3*this->a;
  this->n3.normalize();

  /* fill H matrix
  |4+2D    0    0|
  |   0 4+2D    0|
  |   0    0 4+2D|
  */
  for(int i = 0; i<9; i++)
    this->H[i] = 0.0;
  this->H[0] = 4.0+2*g_const_D; this->H[4] = 4.0+2*g_const_D; this->H[8] = 4.0+2*g_const_D;

  /* fill first 4 lines of A matrix
  |[      n_1^T      ]|
  |[      n_2^T      ]|
  |[      n_3^T      ]|
  |[      a_N^T      ]|
  |[    n_right^T    ]|
  |[     n_left^T    ]|
  |[  n^cam_lower^T  ]|
  |[  n^cam_upper^T  ]|
  */
  for(int i = 0; i<24; i++)
    this->A[i] = 0.0;
  this->A[0] = this->n1[0]; this->A[1] = this->n1[1]; this->A[2] = this->n1[2];
  this->A[3] = this->n2[0]; this->A[4] = this->n2[1]; this->A[5] = this->n2[2];
  this->A[6] = this->n3[0]; this->A[7] = this->n3[1]; this->A[8] = this->n3[2];
  this->A[9] = this->aabs[0]; this->A[10] = this->aabs[1]; this->A[11] = this->aabs[2];

  this->d[0] = 0.0; this->d[1] = 0.0; this->d[2] = 0.0;

  /* fill constant elements of lbA vector
  |          n_1^T*x_1        |
  |          n_2^T*x_2        |
  |          n_3^T*x_3        |
  |      a_N^T*x_1+d_min      |
  |        n_right^T*m        |
  |         n_left^T*m        |
  | n^cam_lower^T*x^rel_lower |
  | n^cam_upper^T*x^rel_upper |
  */
  this->lbA[0] = this->n1.dot(this->x1);
  this->lbA[1] = this->n2.dot(this->x2);
  this->lbA[2] = this->n3.dot(this->x3);
  this->lbA[3] = this->aabs.dot(this->x1+this->aabs*this->minDist);

  /* fill constant elements of ubA vector
  |            inf            |
  |            inf            |
  |            inf            |
  |      a_N^T*x_1+d_max      |
  |            inf            |
  |            inf            |
  |            inf            |
  |            inf            |
  */
  this->ubA[0] = FLT_MAX;
  this->ubA[1] = FLT_MAX;
  this->ubA[2] = FLT_MAX;
  this->ubA[3] = this->aabs.dot(this->x1+this->aabs*this->maxDist);

  Options options;
  this->VPSolver->setOptions( options );
}

template<class System_t, class State_t, class Vector_t, class region_t>
bool Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>::isVisible(Vector_t s)
{
  Vector3f st(s[0], s[1], s[2]);
  if(this->aabs.dot(st-this->x1-this->aabs*this->minDist)<0)
    return false;
  if(this->aabs.dot(st-this->x1-this->aabs*this->maxDist)>0)
    return false;
  if((st-this->x1).dot(this->n1)<0)
    return false;
  if((st-this->x2).dot(this->n2)<0)
    return false;
  if((st-this->x3).dot(this->n3)<0)
    return false;
  for(int it = 0; it<camBoundNormal.size(); it++) {
    Vector3f camN = camBoundRotated(camBoundNormal[it], 0.0, s[3]);
    if(camN.dot(this->x1 - st)<0)
      return false;
    if(camN.dot(this->x2 - st)<0)
      return false;
    if(camN.dot(this->x3 - st)<0)
      return false;
  }
  return !this->IsInCollision(s);
}

template<class System_t, class State_t, class Vector_t, class region_t>
Vector_t Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>::dualBarrierSampling(Vector_t* state1, Vector_t* state2, Vector_t* statePrev)
{
  bool solFound = false;
  bool orSolFound = false;
#ifdef __TIMING_INFO__
  timeval time;
  gettimeofday(&time, NULL);
  time_DBS -= time.tv_sec * 1000000 + time.tv_usec;
#endif
  if(this->Fixpoint) {
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
    Vector_t ret;
    for(int i = 0; i<3; i++)
    {
      ret[i] = this->x1[i];
    }
    ret[3] = this->x2[0];
    return ret;
  }
  double DD = 0.5*(this->maxDist+this->minDist);
  assert(DD<this->maxDist);
  StateVector best;
  for(int i = 0; i<DIMENSIONALITY; i++)
    best[i] = 0;
  double cost = DBL_MAX;
  double angleLower = g_camPitch+g_camAngleVertical/2.0;
  double angleUpper = g_camPitch-g_camAngleVertical/2.0;
  int maxPW = 12;
  double psiInc = 2*M_PI/maxPW;
  Vector3f m = (this->x1+this->x2+this->x3)/3;
  int obsCount = 0;
  for(int pw = 0; pw<maxPW; pw++) // rotate constraints to find possible viewpoints in all directions
  {
    double psiD = pw*psiInc;
    Vector3f right(-sin(psiD-psiInc/2.0), cos(psiD-psiInc/2.0),0.0);
    Vector3f left(-sin(psiD+psiInc/2.0), cos(psiD+psiInc/2.0),0.0);
    Vector3f lowerCam(sin(angleLower)*cos(psiD), sin(angleLower)*sin(psiD), -cos(angleLower));
    Vector3f low = this->x1;
    if(low.dot(lowerCam)<this->x2.dot(lowerCam))
      low = this->x2;
    if(low.dot(lowerCam)<this->x3.dot(lowerCam))
      low = this->x3;
    Vector3f upperCam(-sin(angleUpper)*cos(psiD), -sin(angleUpper)*sin(psiD), cos(angleUpper));
    Vector3f high = this->x1;
    if(high.dot(upperCam)<this->x2.dot(upperCam))
      high = this->x2;
    if(high.dot(upperCam)<this->x3.dot(upperCam))
      high = this->x3;

    /* fill elements 5-8 of lbA vector
    |          n_1^T*x_1        |
    |          n_2^T*x_2        |
    |          n_3^T*x_3        |
    |      a_N^T*x_1+d_min      |
    |        n_right^T*m        |
    |         n_left^T*m        |
    | n^cam_lower^T*x^rel_lower |
    | n^cam_upper^T*x^rel_upper |
    */
    this->lbA[4] = right.dot(m);
    this->lbA[5] = left.dot(m);
    this->lbA[6] = lowerCam.dot(low);
    this->lbA[7] = upperCam.dot(high);
    /* fill elements 5-8 of ubA vector
    |            inf            |
    |            inf            |
    |            inf            |
    |      a_N^T*x_1+d_max      |
    |            inf            |
    |            inf            |
    |            inf            |
    |            inf            |
    */
    this->ubA[4] = FLT_MAX;
    this->ubA[5] = FLT_MAX;
    this->ubA[6] = FLT_MAX;
    this->ubA[7] = FLT_MAX;

    /* fill lines 5-8 of A matrix
    |[      n_1^T      ]|
    |[      n_2^T      ]|
    |[      n_3^T      ]|
    |[      a_N^T      ]|
    |[    n_right^T    ]|
    |[     n_left^T    ]|
    |[  n^cam_lower^T  ]|
    |[  n^cam_upper^T  ]|
    */
    this->A[12] = right[0];
    this->A[13] = right[1];
    this->A[14] = right[2];

    this->A[15] = left[0];
    this->A[16] = left[1];
    this->A[17] = left[2];

    this->A[18] = lowerCam[0];
    this->A[19] = lowerCam[1];
    this->A[20] = lowerCam[2];

    this->A[21] = upperCam[0];
    this->A[22] = upperCam[1];
    this->A[23] = upperCam[2];

    StateVector g; g << (this->x1[0]+this->x2[0]+this->x3[0])/3+DD*this->aabs[0],
                        (this->x1[1]+this->x2[1]+this->x3[1])/3+DD*this->aabs[1],
                        (this->x1[2]+this->x2[2]+this->x3[2])/3+DD*this->aabs[2], 0.0;

    static real_t lbx[3] = { X_MIN, Y_MIN, Z_MIN };
    static real_t ubx[3] = { X_MAX, Y_MAX, Z_MAX };
    int nWSR = 100;
    returnValue re;
    bool solFoundLocal = false;
    /* fill vector d
    ||          |   |           |   |           ||
    || -2Dg^k-1 | + | -2g_p^k-1 | + | -2g_s^k-1 ||
    ||          |   |           |   |           ||
    */
    if(this->initialized)
    {
      this->d[0] = -2.0*g_const_D*(*statePrev)[0]-2.0*(*state1)[0]-2.0*(*state2)[0];
      this->d[1] = -2.0*g_const_D*(*statePrev)[1]-2.0*(*state1)[1]-2.0*(*state2)[1];
      this->d[2] = -2.0*g_const_D*(*statePrev)[2]-2.0*(*state1)[2]-2.0*(*state2)[2];
      if(SUCCESSFUL_RETURN != (re = this->VPSolver->init( this->H,this->d,this->A,lbx,ubx,this->lbA,this->ubA, nWSR )))
      {
        continue;
      }
      else
      {
        solFoundLocal = true;
      }
    }
    else
    {
      this->d[0] = -(4.0+2.0*g_const_D)*g[0];
      this->d[1] = -(4.0+2.0*g_const_D)*g[1];
      this->d[2] = -(4.0+2.0*g_const_D)*g[2];
      if(SUCCESSFUL_RETURN != (re = this->VPSolver->init( this->H,this->d,this->A,lbx,ubx,this->lbA,this->ubA, nWSR ))) 
      {
        continue;
      }
      else
      {
        solFoundLocal = true;
      }
    }

    /* compute the constant term of the optimization objective
    g_p^k-1^T*g_p^k-1 + g_s^k-1^T*g_s^k-1 + D*g^k-1^T*g^k-1
    */
    double xxCompensate = 0.0;
    if(this->initialized)
    {
      for(int k = 0; k<3; k++)
        xxCompensate += pow((*state1)[k],2.0);
      for(int k = 0; k<3; k++)
        xxCompensate += pow((*state2)[k],2.0);
      for(int k = 0; k<3; k++)
        xxCompensate += g_const_D*pow((*statePrev)[k],2.0);
    }
    else
    {
      for(int k = 0; k<3; k++)
        xxCompensate += pow(g[k],2.0);
      xxCompensate *= 4.0 + 2.0*g_const_D;
    }
    
    real_t xOpt[3];
    this->VPSolver->getPrimalSolution( xOpt );

    g[0] = xOpt[0];
    g[1] = xOpt[1];
    g[2] = xOpt[2];

    region_t* obs = NULL;
    // obstacle detection and avoidance
    if(obs = this->IsInCollision(g))
    {
      solFoundLocal = false;
      real_t lb[3] = { X_MIN, Y_MIN, Z_MIN };
      real_t ub[3] = { X_MAX, Y_MAX, Z_MAX };
      if(0.0>this->AvoidObstacle(lb, ub, 1, obs, xxCompensate, g))
      {
        obsCount++;
      }
      else
      {
        solFoundLocal = true;
      }
    }

    double costOrientation = DBL_MAX;
    double dp = 1e-9;
    double ds = 1e-9;
    double alfa1 = 0.0;
    double alfa2 = 0.0;
    if(this->initialized)
    {
      dp += sqrt(pow((*state1)[0] - g[0],2.0) + pow((*state1)[1] - g[1],2.0) + pow((*state1)[2] - g[2],2.0));
      ds += sqrt(pow((*state2)[0] - g[0],2.0) + pow((*state2)[1] - g[1],2.0) + pow((*state2)[2] - g[2],2.0));
      alfa1 = atan2(g[1]-(*state1)[1], g[0]-(*state1)[0]);
      alfa2 = atan2((*state2)[1]-g[1], (*state2)[0]-g[0]);
    }
    for(double psi = -M_PI; psi<M_PI; psi+=g_angular_discretization_step)
    {
      StateVector s = g;
      s[3] = psi;
      double c = 0.9*DBL_MAX;
      if(this->initialized)
      {
        double dpsi1 = s[3]-alfa1;
        double dpsi2 = s[3]-alfa2;
        if(fabs(dpsi1)>M_PI)
          dpsi1 = 2*M_PI-fabs(dpsi1);
        if(fabs(dpsi2)>M_PI)
          dpsi2 = 2*M_PI-fabs(dpsi2);
        c = pow(dpsi1,2.0)/dp+pow(dpsi2, 2.0)/ds;
      }
      if(c<=costOrientation && this->isVisible(s))
      {
        g[3] = s[3];
        costOrientation = c;
        orSolFound = true;
      }
    } 
    if(this->VPSolver->getObjVal()+xxCompensate+costOrientation<cost && solFoundLocal)
    {
      best = g;
      cost = this->VPSolver->getObjVal()+xxCompensate+costOrientation;
    }
    solFound |= solFoundLocal;
  }
  if(obsCount>=maxPW)
  {
    std::string pkgPath = ros::package::getPath("koptplanner");
    std::fstream plannerLog;
    plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
    if(!plannerLog.is_open())
      ROS_ERROR("Could not open report.log");
    plannerLog << "-->Too may obstacles in the dual sampling space!\n";
    plannerLog << "   x1: (" << this->x1[0] << ", " << this->x1[1] << ", " << this->x1[2] << ")\n";
    plannerLog << "   x2: (" << this->x2[0] << ", " << this->x2[1] << ", " << this->x2[2] << ")\n";
    plannerLog << "   x3: (" << this->x3[0] << ", " << this->x3[1] << ", " << this->x3[2] << ")\n";
    plannerLog.close();
    koptError = OBSTACLE_INFEASIBILITY;
  }

  if(!solFound)
  {
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
    if(this->initialized)
      return (*statePrev);
    else
    {
      std::string pkgPath = ros::package::getPath("koptplanner");
      std::fstream plannerLog;
      plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
      if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
      plannerLog << "-->No feasible position to inspect triangle:\n";
      plannerLog << "   x1: (" << this->x1[0] << ", " << this->x1[1] << ", " << this->x1[2] << ")\n";
      plannerLog << "   x2: (" << this->x2[0] << ", " << this->x2[1] << ", " << this->x2[2] << ")\n";
      plannerLog << "   x3: (" << this->x3[0] << ", " << this->x3[1] << ", " << this->x3[2] << ")\n";
      plannerLog.close();
    }
    koptError = VIEWPOINT_INFEASIBILITY;
  }
  if(!orSolFound)
  {
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
    if(this->initialized)
      return (*statePrev);
    else
    {
      std::string pkgPath = ros::package::getPath("koptplanner");
      std::fstream plannerLog;
      plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
      if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
      plannerLog << "-->No feasible heading to inspect triangle:\n";
      plannerLog << "   x1: (" << this->x1[0] << ", " << this->x1[1] << ", " << this->x1[2] << ")\n";
      plannerLog << "   x2: (" << this->x2[0] << ", " << this->x2[1] << ", " << this->x2[2] << ")\n";
      plannerLog << "   x3: (" << this->x3[0] << ", " << this->x3[1] << ", " << this->x3[2] << ")\n";
      plannerLog.close();
    }
    koptError = VIEWPOINT_INFEASIBILITY;
  }
  for(int i = 0; i<4; i++)
    assert(best[i]<1e15&& best[i]>-1e15);
#ifdef __TIMING_INFO__
  gettimeofday(&time, NULL);
  time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
  return best;
}

template<class System_t, class State_t, class Vector_t, class region_t>
double Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>::AvoidObstacle(real_t* lb, real_t* ub, int Nobs, region_t* obs, double xxCompensate, StateVector& g)
{
  double opt = DBL_MAX;
  real_t xOpt[3];
  if(Nobs>g_max_obstacle_depth || ros::isShuttingDown()) {
    return -1.0;
  }
  real_t lb_l[8] = {lb[0], lb[1], lb[2], lb[3], lb[4], lb[5], lb[6], lb[7]};
  real_t ub_l[8] = {ub[0], ub[1], ub[2], ub[3], ub[4], ub[5], ub[6], ub[7]};
  // -- min --
  for(int i = 0; i<3; i++)
  {
    int nWSR = 100;
    lb_l[i] = std::max(obs->center[i]+obs->size[i]/2+g_security_distance, lb_l[i]);
    if(SUCCESSFUL_RETURN == this->VPSolver->hotstart( this->d,lb_l,ub_l,this->lbA,this->ubA, nWSR ))
    {
      if(this->VPSolver->getObjVal()+xxCompensate<opt)
      {
        if(RET_QP_NOT_SOLVED == this->VPSolver->getPrimalSolution( xOpt ))
          continue;
        StateVector gPot = g;
        this->VPSolver->getPrimalSolution( xOpt );
        gPot[0] = xOpt[0]; gPot[1] = xOpt[1]; gPot[2] = xOpt[2];
        region_t* obsPot = NULL;
        if(obsPot = this->IsInCollision(gPot))
        {
          double optPot = DBL_MAX;
          if(0.0<(optPot = this->AvoidObstacle(lb_l, ub_l, Nobs+1, obsPot, xxCompensate, gPot)))
          {
            opt = optPot;
            g = gPot;
          }
        }
        else
        {
          opt = this->VPSolver->getObjVal()+xxCompensate;
          g = gPot;
        }
      }
    }
    lb_l[i] = lb[i];
  }
  // -- max --
  for(int i = 0; i<3; i++)
  {
    int nWSR = 100;
    ub_l[i] = std::min(obs->center[i]-obs->size[i]/2-g_security_distance, ub_l[i]);
    if(SUCCESSFUL_RETURN == this->VPSolver->hotstart( this->d,lb_l,ub_l,this->lbA,this->ubA, nWSR ))
    {
      if(this->VPSolver->getObjVal()+xxCompensate<opt)
      {
        if(RET_QP_NOT_SOLVED == this->VPSolver->getPrimalSolution( xOpt ))
          continue;
        StateVector gPot = g;
        this->VPSolver->getPrimalSolution( xOpt );
        gPot[0] = xOpt[0]; gPot[1] = xOpt[1]; gPot[2] = xOpt[2];
        region_t* obsPot = NULL;
        if(obsPot = this->IsInCollision(gPot))
        {
          double optPot = DBL_MAX;
          if(0.0<(optPot = this->AvoidObstacle(lb_l, ub_l, Nobs+1, obsPot, xxCompensate, gPot)))
          {
            opt = optPot;
            g = gPot;
          }
        }
        else
        {
          opt = this->VPSolver->getObjVal()+xxCompensate;
          g = gPot;
        }
        
      }
    }
    ub_l[i] = ub[i];
  }
  if(opt>0.0)
    return opt;
  else
    return -1.0;
}

template<class State_t>
Rotorcraft::Trajectory<State_t>::Trajectory ()
{

}


template<class State_t>
Rotorcraft::Trajectory<State_t>::~Trajectory ()
{

}


template<class State_t>
Rotorcraft::Trajectory<State_t>::Trajectory (const Rotorcraft::Trajectory<State_t> &trajectoryIn)
{
  this->endState = new State_t (trajectoryIn.getEndState());
}

template<class Trajectory_t, class State_t, class region_t>
Rotorcraft::System<Trajectory_t, State_t, region_t>::System ()
{
  this->numDimensions = 0;
}


template<class Trajectory_t, class State_t, class region_t>
Rotorcraft::System<Trajectory_t, State_t, region_t>::~System ()
{
    
}

template<class Trajectory_t, class State_t, class region_t>
int Rotorcraft::System<Trajectory_t, State_t, region_t>::extendTo (State_t &stateFromIn, State_t &stateTowardsIn, Trajectory_t &trajectoryOut, bool &exactConnectionOut)
{
  double *dists = new double[this->numDimensions];
  for (int i = 0; i < this->numDimensions; i++) 
    dists[i] = stateTowardsIn.x[i] - stateFromIn.x[i];
  
  double distTotal = 0.0;
  for (int i = 0; i < this->numDimensions; i++) 
    distTotal += dists[i]*dists[i];
  distTotal = sqrt (distTotal);
  
  double incrementTotal = distTotal/g_discretization_step;
  // normalize the distance according to the discretization step
  for (int i = 0; i < this->numDimensions; i++)
    dists[i] /= incrementTotal;
  
  int numSegments = (int)floor(incrementTotal);
  
  double *stateCurr = new double[this->numDimensions];
  for (int i = 0; i < this->numDimensions; i++) 
    stateCurr[i] = stateFromIn.x[i];
  
  for (int i = 0; i < numSegments; i++)
  {
    if(this->IsInCollision (stateCurr))
    {
      trajectoryOut.totalVariation = FLT_MAX;
      return 0;
    }
    
    for (int i = 0; i < this->numDimensions; i++)
      stateCurr[i] += dists[i];
  }
  
  if(this->IsInCollision (stateTowardsIn.x))
  {
    trajectoryOut.totalVariation = FLT_MAX;
    return 0;
  }
  
  if(trajectoryOut.endState)
    delete trajectoryOut.endState;
  trajectoryOut.endState = new State_t (stateTowardsIn);
  trajectoryOut.totalVariation = distTotal;
  
  delete [] dists;
  delete [] stateCurr;
  
  exactConnectionOut = true;
  
  return 1;
}

template<class Trajectory_t, class State_t, class region_t>
double Rotorcraft::System<Trajectory_t, State_t, region_t>::evaluateExtensionCost (State_t& stateFromIn, State_t& stateTowardsIn, bool &exactConnectionOut)
{
  exactConnectionOut = true;
  
  double distTotal = 0.0;
  for (int i = 0; i < this->numDimensions-1; i++)
  {
    double distCurr = stateTowardsIn.x[i] - stateFromIn.x[i];
    distTotal += distCurr*distCurr;
  }
  return sqrt(distTotal);
}

template<class System_t, class State_t, class Vector_t, class region_t>
void Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>::setCamBoundNormals()
{
  camBoundNormal.clear();
  Vector3f top(-sin(g_camPitch-g_camAngleVertical/2), 0, -cos(g_camPitch-g_camAngleVertical/2));
  camBoundNormal.push_back(top);
  Vector3f bottom(sin(g_camPitch+g_camAngleVertical/2), 0, cos(g_camPitch+g_camAngleVertical/2));
  camBoundNormal.push_back(bottom);
  Vector3f axis(sin(g_camPitch), 0, cos(g_camPitch));
  AngleAxisf m1 = AngleAxisf(g_camAngleHorizontal/2.0, axis);
  Vector3f y1(0, -1, 0);
  Vector3f left = m1*y1;
  camBoundNormal.push_back(left);
  AngleAxisf m2 = AngleAxisf(-g_camAngleHorizontal/2.0, axis);
  Vector3f y2(0, 1, 0);
  Vector3f right = m2*y2;
  camBoundNormal.push_back(right);
}

template<class System_t, class State_t, class Vector_t, class region_t>
Vector3f Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>::camBoundRotated(Vector3f normal, double roll, double yaw)
{
  Vector3f x(1, 0, 0);
  Vector3f z(0, 0, 1);
  AngleAxisf mroll = AngleAxisf(roll, x);
  AngleAxisf myaw = AngleAxisf(yaw, z);
  return myaw*(mroll*normal);
}

#endif
