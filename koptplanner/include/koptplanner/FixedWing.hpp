/*!
 * \file FixedWing.hpp
 *
 * More elaborate description
 */
#ifndef __FIXEDWING_HPP__
#define __FIXEDWING_HPP__

#include "ros/ros.h"
#include "koptplanner/FixedWing.h"
#include "koptplanner/plan.hpp"
#include "koptplanner/ptpPlanner.hpp"
#include "float.h"
#include <Eigen/Geometry>
#include "optec/qpOASES.hpp"
#include <ros/package.h>
#include <fstream>

#define FIXEDWING_SYSTEM FixedWing::System<FixedWing::Trajectory<FixedWing::State>, FixedWing::State, FixedWing::region>
#define SQ(x) ((x)*(x))
#define X_MIN_B (problemBoundary.center[0]-problemBoundary.size[0]/2+FIXEDWING_SYSTEM::r_min)
#define Y_MIN_B (problemBoundary.center[1]-problemBoundary.size[1]/2+FIXEDWING_SYSTEM::r_min)
#define Z_MIN_B (problemBoundary.center[2]-problemBoundary.size[2]/2)
#define X_MAX_B (problemBoundary.center[0]+problemBoundary.size[0]/2-FIXEDWING_SYSTEM::r_min)
#define Y_MAX_B (problemBoundary.center[1]+problemBoundary.size[1]/2-FIXEDWING_SYSTEM::r_min)
#define Z_MAX_B (problemBoundary.center[2]+problemBoundary.size[2]/2)

#define BASE_LINE_FACTOR 1.5

#ifdef __TIMING_INFO__
 extern long time_DBS;
 extern long time_RRTS;
 extern long time_RRTS_req;
 extern long time_LKH;
#endif

extern double g_camAngleHorizontal;
extern double g_camAngleVertical;
extern double g_camPitch;
extern double g_maxClimbSinkRate;
extern double ** lookupTable;
extern koptError_t koptError;
extern MDL_NAMESPACE::region problemBoundary;

extern double g_const_A;
extern double g_const_B;
extern double g_const_C;
extern double g_const_D;
extern int g_max_obstacle_depth;
extern double g_discretization_step;
extern double g_angular_discretization_step;
extern double g_security_distance;

template<class Trajectory_t, class State_t, class region_t>
double FixedWing::System<Trajectory_t, State_t, region_t>::r_min = 0.0;

template<class System_t, class State_t, class Vector_t, class region_t>
FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::Triangle()
{
  // x = [x, y, z, q1, q2, q3, slp, sls]
  this->Fixpoint = false;
  this->VPSolver = new QProblem( 8,13 );
  this->H = new real_t[8*8];
  this->A = new real_t[13*8];
  this->d = new real_t[8];
  this->lbA = new real_t[13];
  this->ubA = new real_t[13];
}

template<class System_t, class State_t, class Vector_t, class region_t>
void FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::init()
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

  for(int i = 0; i<64; i++)
    this->H[i] = 0.0;
  /* fill diagonal of H matrix
  |2D 0  0     0 0 0 0 0|
  |0  2D 0     0 0 0 0 0|
  |0  0  2D+4A 0 0 0 0 0|
  |0  0  0     2 0 0 0 0|
  |0  0  0     0 2 0 0 0|
  |0  0  0     0 0 2 0 0|
  |0  0  0     0 0 0 0 0|
  |0  0  0     0 0 0 0 0|
  */
  this->H[0] = 2.0*g_const_D + 4.0*g_const_B; this->H[9] = 2.0*g_const_D + 4.0*g_const_B; this->H[18] = 2.0*g_const_D + 4.0*g_const_A + 4.0*g_const_B; this->H[27] = 2.0; this->H[36] = 2.0; this->H[45] = 2.0; this->H[54] = 0.0; this->H[63] = 0.0;

  /* fill first 4 lines of A matrix
  |[      n_1^T      ]  0  0  0  0  0|
  |[      n_2^T      ]  0  0  0  0  0|
  |[      n_3^T      ]  0  0  0  0  0|
  |[      a_N^T      ]  0  0  0  0  0|
  |[    n_right^T    ]  0  0  0  0  0|
  |[     n_left^T    ]  0  0  0  0  0|
  |[  n^cam_lower^T  ]  0  0  0  0  0|
  |[  n^cam_upper^T  ]  0  0  0  0  0|
  |[ gk-1^T-gk-1_p^T ]  0  0  0  1  0|
  |[ gk-1^T-gk-1_s^T ]  0  0  0  0  1|
  ||                 | -1  0  0  0  0|
  ||        B        |  0 -1  0  0  0|
  ||                 |  0  0 -1  0  0|
  */
  for(int i = 0; i<104; i++)
    this->A[i] = 0.0;
  this->A[0] = this->n1[0]; this->A[1] = this->n1[1]; this->A[2] = this->n1[2];
  this->A[8] = this->n2[0]; this->A[9] = this->n2[1]; this->A[10] = this->n2[2];
  this->A[16] = this->n3[0]; this->A[17] = this->n3[1]; this->A[18] = this->n3[2];
  this->A[24] = this->aabs[0]; this->A[25] = this->aabs[1]; this->A[26] = this->aabs[2];

  /* fill vector d
  ||          |            |
  || -2Dg^k-1 |            |
  ||          | -2A(zp+zs) |
  |            0           |
  |            0           |
  |            0           |
  |            C           |
  |            C           |
  */
  for(int i = 0; i<8; i++)
    this->d[i] = 0.0;
  this->d[6] = g_const_C;
  this->d[7] = g_const_C;

  /* fill constant elements of lbA vector
  |          n_1^T*x_1        |
  |          n_2^T*x_2        |
  |          n_3^T*x_3        |
  |      a_N^T*x_1+d_min      |
  |        n_right^T*m        |
  |         n_left^T*m        |
  | n^cam_lower^T*x^rel_lower |
  | n^cam_upper^T*x^rel_upper |
  |            l_p²           |
  |            l_s²           |
  ||                         ||
  ||         B*g^k-1_p       ||
  ||                         ||
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
  |            inf            |
  |            inf            |
  ||                         ||
  ||         B*g^k-1_p       ||
  ||                         ||
  */
  this->ubA[0] = FLT_MAX;
  this->ubA[1] = FLT_MAX;
  this->ubA[2] = FLT_MAX;
  this->ubA[3] = this->aabs.dot(this->x1+this->aabs*this->maxDist);

  Options options;
  this->VPSolver->setOptions( options );
}

template<class System_t, class State_t, class Vector_t, class region_t>
bool FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::isVisible(Vector_t s)
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
  for(int it = 0; it<camBoundNormal.size(); it++)
  {
    Vector3f camN = camBoundRotated(camBoundNormal[it], s[3], s[5]);
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
Vector_t FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::dualBarrierSampling(Vector_t* state1, Vector_t* state2, Vector_t* statePrev)
{
  bool solFound = false;
  bool orSolFound = false;
#ifdef __TIMING_INFO__
  timeval time;
  gettimeofday(&time, NULL);
  time_DBS -= time.tv_sec * 1000000 + time.tv_usec;
#endif
  if(this->Fixpoint)
  {
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
    Vector_t ret;
    for(int i = 0; i<3; i++)
    {
      ret[i] = this->x1[i];
      ret[i+3] = this->x2[i];
    }
    return ret;
  }

  if(this->initialized) // compute base-line criterion
  {
    double alph1 = atan2((*statePrev)[1] - (*state1)[1], (*statePrev)[0] - (*state1)[0]);
    double alph2 = atan2((*state2)[1] - (*statePrev)[1], (*state2)[0] - (*statePrev)[0]);
    double a1 = fmod(alph1 - (*state1)[5]+4*M_PI,2*M_PI);
    double dpsi1 = fmod((*statePrev)[5] - (*state1)[5]+4*M_PI,2*M_PI);
    double a2 = fmod(alph2 - (*statePrev)[5]+4*M_PI,2*M_PI);
    double dpsi2 = fmod((*state2)[5] - (*statePrev)[5]+4*M_PI,2*M_PI);
    double N = LOOKUPTABLE_SIZE;
    double baseLineMin1 = pow( BASE_LINE_FACTOR*lookupTable[(int)floor(N*a1/(2*M_PI))][(int)floor(N*dpsi1/(2*M_PI))]*FIXEDWING_SYSTEM::r_min, 2.0 );
    double baseLineMin2 = pow( BASE_LINE_FACTOR*lookupTable[(int)floor(N*a2/(2*M_PI))][(int)floor(N*dpsi2/(2*M_PI))]*FIXEDWING_SYSTEM::r_min, 2.0 );
    baseLineMin1 += (*state1)[0]*((*statePrev)[0] - (*state1)[0]) + (*state1)[1]*((*statePrev)[1] - (*state1)[1]) + (*state1)[2]*((*statePrev)[2] - (*state1)[2]);
    baseLineMin2 += (*state2)[0]*((*statePrev)[0] - (*state2)[0]) + (*state2)[1]*((*statePrev)[1] - (*state2)[1]) + (*state2)[2]*((*statePrev)[2] - (*state2)[2]);
    /* fill lines 9 & 10 of A matrix
    |[      n_1^T      ]  0  0  0  0  0|
    |[      n_2^T      ]  0  0  0  0  0|
    |[      n_3^T      ]  0  0  0  0  0|
    |[      a_N^T      ]  0  0  0  0  0|
    |[    n_right^T    ]  0  0  0  0  0|
    |[     n_left^T    ]  0  0  0  0  0|
    |[  n^cam_lower^T  ]  0  0  0  0  0|
    |[  n^cam_upper^T  ]  0  0  0  0  0|
    |[ gk-1^T-gk-1_p^T ]  0  0  0  1  0|
    |[ gk-1^T-gk-1_s^T ]  0  0  0  0  1|
    ||                 | -1  0  0  0  0|
    ||        B        |  0 -1  0  0  0|
    ||                 |  0  0 -1  0  0|
    */
    this->A[64] = (*statePrev)[0] - (*state1)[0];
    this->A[65] = (*statePrev)[1] - (*state1)[1];
    this->A[66] = (*statePrev)[2] - (*state1)[2];
    this->A[70] = 1.0;
    this->A[72] = (*statePrev)[0] - (*state2)[0];
    this->A[73] = (*statePrev)[1] - (*state2)[1];
    this->A[74] = (*statePrev)[2] - (*state2)[2];
    this->A[79] = 1.0;
    /* fill corresponding elements of lbA vector
    |          n_1^T*x_1        |
    |          n_2^T*x_2        |
    |          n_3^T*x_3        |
    |      a_N^T*x_1+d_min      |
    |        n_right^T*m        |
    |         n_left^T*m        |
    | n^cam_lower^T*x^rel_lower |
    | n^cam_upper^T*x^rel_upper |
    |            l_p²           |
    |            l_s²           |
    ||                         ||
    ||         B*g^k-1_p       ||
    ||                         ||
    */
    this->lbA[8] = baseLineMin1;
    this->lbA[9] = baseLineMin2;
    /* fill corresponding elements of ubA vector
    |            inf            |
    |            inf            |
    |            inf            |
    |      a_N^T*x_1+d_max      |
    |            inf            |
    |            inf            |
    |            inf            |
    |            inf            |
    |            inf            |
    |            inf            |
    ||                         ||
    ||         B*g^k-1_p       ||
    ||                         ||
    */
    this->ubA[8] = FLT_MAX;
    this->ubA[9] = FLT_MAX;
  }
  else
  {
    this->A[70] = 1.0;
    this->A[79] = 1.0;
    this->lbA[8] = -FLT_MAX;
    this->lbA[9] = -FLT_MAX;
    this->ubA[8] = FLT_MAX;
    this->ubA[9] = FLT_MAX;
  }

  double DD = 0.5*(this->maxDist+this->minDist);
  assert(DD<this->maxDist);
  StateVector best;
  for(int i = 0; i<DIMENSIONALITY; i++)
    best[i] = 0;
  double cost = DBL_MAX;
  double angleLower = g_camPitch+g_camAngleVertical/2.0 - 5*M_PI/180;
  double angleUpper = g_camPitch-g_camAngleVertical/2.0 + 5*M_PI/180;
  int maxPW = 18;
  double psiInc = 2*M_PI/maxPW;
  Vector3f m = (this->x1+this->x2+this->x3)/3;
  int obstacleCounter = 0;
  for(int pw = 0; pw<maxPW; pw++) // rotate constraints to find possible viewpoints in all directions
  {
    bool solFoundLocal = false;
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
    |            l_p²           |
    |            l_s²           |
    ||                         ||
    ||         B*g^k-1_p       ||
    ||                         ||
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
    |            inf            |
    |            inf            |
    ||                         ||
    ||         B*g^k-1_p       ||
    ||                         ||
    */
    this->ubA[4] = FLT_MAX;
    this->ubA[5] = FLT_MAX;
    this->ubA[6] = FLT_MAX;
    this->ubA[7] = FLT_MAX;
    /* fill lines 5-8 of A matrix and if first tour exists also 11-13
    |[      n_1^T      ]  0  0  0  0  0|
    |[      n_2^T      ]  0  0  0  0  0|
    |[      n_3^T      ]  0  0  0  0  0|
    |[      a_N^T      ]  0  0  0  0  0|
    |[    n_right^T    ]  0  0  0  0  0|
    |[     n_left^T    ]  0  0  0  0  0|
    |[  n^cam_lower^T  ]  0  0  0  0  0|
    |[  n^cam_upper^T  ]  0  0  0  0  0|
    |[ gk-1^T-gk-1_p^T ]  0  0  0  1  0|
    |[ gk-1^T-gk-1_s^T ]  0  0  0  0  1|
    ||                 | -1  0  0  0  0|
    ||        B        |  0 -1  0  0  0|
    ||                 |  0  0 -1  0  0|
    */
    this->A[32] = right[0];
    this->A[33] = right[1];
    this->A[34] = right[2];

    this->A[40] = left[0];
    this->A[41] = left[1];
    this->A[42] = left[2];

    this->A[48] = lowerCam[0];
    this->A[49] = lowerCam[1];
    this->A[50] = lowerCam[2];

    this->A[56] = upperCam[0];
    this->A[57] = upperCam[1];
    this->A[58] = upperCam[2];

    if(this->initialized)
    {
      Vector3f b ( (*state1)[0] - (*state2)[0], (*state1)[1] - (*state2)[1], (*state1)[2] - (*state2)[2] ); b.normalize();
      this->A[80] = 0.0;
      this->A[81] = -b[2];
      this->A[82] = b[1];
      this->A[83] = -1.0;
      this->A[88] = b[2];
      this->A[89] = 0.0;
      this->A[90] = -b[0];
      this->A[92] = -1.0;
      this->A[96] = -b[1];
      this->A[97] = b[0];
      this->A[98] = 0.0;
      this->A[101] = -1.0;
      this->lbA[10] = -(*state1)[1]*b[2] + (*state1)[2]*b[1];
      this->lbA[11] = (*state1)[0]*b[2] - (*state1)[2]*b[0];
      this->lbA[12] = -(*state1)[0]*b[1] + (*state1)[1]*b[0];
      this->ubA[10] = -(*state1)[1]*b[2] + (*state1)[2]*b[1];
      this->ubA[11] = (*state1)[0]*b[2] - (*state1)[2]*b[0];
      this->ubA[12] = -(*state1)[0]*b[1] + (*state1)[1]*b[0];
    }
    else
    {
      // if no tour exists yet adapt condition to: boundary_max >= g^k >= boundary_min, which is always fulfilled
      this->A[80] = 1.0;
      this->A[89] = 1.0;
      this->A[98] = 1.0;
      this->lbA[10] = X_MIN_B;
      this->lbA[11] = Y_MIN_B;
      this->lbA[12] = Z_MIN_B;
      this->ubA[10] = X_MAX_B;
      this->ubA[11] = Y_MAX_B;
      this->ubA[12] = Z_MAX_B;
    }

    StateVector g; g << (this->x1[0]+this->x2[0]+this->x3[0])/3+DD*this->aabs[0],
                        (this->x1[1]+this->x2[1]+this->x3[1])/3+DD*this->aabs[1],
                        (this->x1[2]+this->x2[2]+this->x3[2])/3+DD*this->aabs[2], 0.0, 0.0, 0.0;

    real_t lbx[8] = { X_MIN_B, Y_MIN_B, Z_MIN_B, -FLT_MAX, -FLT_MAX, -FLT_MAX, 0, 0 };
    real_t ubx[8] = { X_MAX_B, Y_MAX_B, Z_MAX_B, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
    int nWSR = 100;
    returnValue re;
    /* fill vector d
    ||          |            |
    || -2Dg^k-1 |            |
    ||          | -2A(zp+zs) |
    |            0           |
    |            0           |
    |            0           |
    |            C           |
    |            C           |
    */
    if(this->initialized)
    {
      this->d[0] = -2.0*(*statePrev)[0]*g_const_D - 2.0*g_const_B*((*state1)[0]+(*state2)[0]);
      this->d[1] = -2.0*(*statePrev)[1]*g_const_D - 2.0*g_const_B*((*state1)[1]+(*state2)[1]);
      this->d[2] = -2.0*(*statePrev)[2]*g_const_D - 2.0*g_const_B*((*state1)[2]+(*state2)[2]) - 2.0*(*state1)[2]*g_const_A -2.0*(*state2)[2]*g_const_A;
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
      // initialization with g as a reference point
      this->d[0] = -2.0*g[0]*(2.0*g_const_B + g_const_D);
      this->d[1] = -2.0*g[1]*(2.0*g_const_B + g_const_D);
      this->d[2] = -2.0*g[2]*(2.0*(g_const_B + g_const_A) + g_const_D);
      if(SUCCESSFUL_RETURN != (re = this->VPSolver->init( this->H,this->d,this->A,lbx,ubx,this->lbA,this->ubA, nWSR )))
      {
        continue;
      }
      else
      {
        solFoundLocal = true;
      }
    }

    /* calculate constant term
    Dg^k-1^T*g^k-1 + A(zp^2+zs^2)
    */
    double xxCompensate = 0.0;
    if(this->initialized)
    {
      Vector3f d2Comp((*statePrev)[0],(*statePrev)[1],(*statePrev)[2]);
      Vector3f b1Comp((*state1)[0],(*state1)[1],(*state1)[2]);
      Vector3f b2Comp((*state2)[0],(*state2)[1],(*state2)[2]);
      xxCompensate = d2Comp.dot(d2Comp)*g_const_D + b1Comp.dot(b1Comp)*g_const_B + b2Comp.dot(b2Comp)*g_const_B + g_const_A*(pow((*state1)[2],2.0)+pow((*state2)[2],2.0));
    }
    else
    {
      Vector3f d2Comp(g[0],g[1],g[2]);
      xxCompensate = d2Comp.dot(d2Comp)*(2.0*g_const_B+g_const_D) + g_const_A*2.0*pow(g[2],2.0);
    }

    real_t xOpt[8];
    this->VPSolver->getPrimalSolution( xOpt );

    g[0] = xOpt[0];
    g[1] = xOpt[1];
    g[2] = xOpt[2];

    region_t* obs = NULL;
    // obstacle detection and avoidance
    if(obs = this->IsInCollision(g))
    {
      solFoundLocal = false;
      real_t lb[8] = { X_MIN_B, Y_MIN_B, Z_MIN_B, -FLT_MAX, -FLT_MAX, -FLT_MAX, 0, 0};
      real_t ub[8] = { X_MAX_B, Y_MAX_B, Z_MAX_B, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
      double re;
      if(DBL_MAX==(re = this->AvoidObstacle(lb, ub, 1, obs, xxCompensate, g)))
      {
        obstacleCounter++;
      }
      else
      {
        solFoundLocal = true;
      }
    }
    // approximate pitch angle at viewpoint
    double dz = 0;
    if(this->initialized)
    {
      double dist1 = sqrt( pow((*state1)[0] - g[0], 2.0) + pow((*state1)[1] - g[1], 2.0) );
      if(dist1 != 0)
        dz = -atan2(((*state1)[2] - g[2]), dist1);
      double dist2 = sqrt( pow((*state2)[0] - g[0], 2.0) + pow((*state2)[1] - g[1], 2.0) );
      if(dist2 != 0)
        dz += atan2(((*state2)[2] - g[2]), dist2);
    }
    dz/=2.0;
    if(dz > g_maxClimbSinkRate)
      dz = g_maxClimbSinkRate;
    if(dz < -g_maxClimbSinkRate)
      dz = -g_maxClimbSinkRate;
    g[4] = dz;

    double costOrientation = DBL_MAX;
    double dp = 1e-9;
    double ds = 1e-9;
    if(this->initialized)
    {
      dp += sqrt(pow((*state1)[0] - g[0],2.0) + pow((*state1)[1] - g[1],2.0) + pow((*state1)[2] - g[2],2.0));
      ds += sqrt(pow((*state2)[0] - g[0],2.0) + pow((*state2)[1] - g[1],2.0) + pow((*state2)[2] - g[2],2.0));
    }
    for(double psi = -M_PI; psi<M_PI; psi+=g_angular_discretization_step)
    {
      StateVector s = g;
      s[3] = 0.0;
      s[5] = psi;
      double c = 0.9*DBL_MAX;
      if(this->initialized)
      {
        double alfa1 = atan2(g[1]-(*state1)[1], g[0]-(*state1)[0]);
        double alfa2 = atan2((*state2)[1]-g[1], (*state2)[0]-g[0]);
        c = pow(s[5]-alfa1,2.0)/dp+pow(s[5]-alfa2, 2.0)/ds+fabs(s[3]);
        // damping:
        c+= pow(s[5]-(*statePrev)[5], 2.0);
        c*=5e2;
        double N = LOOKUPTABLE_SIZE;
        double a1 = fmod(alfa1 - (*state1)[5]+4*M_PI,2*M_PI);
        double dpsi1 = fmod((*statePrev)[5] - (*state1)[5]+4*M_PI,2*M_PI);
        double a2 = fmod(alfa2 - (*statePrev)[5]+4*M_PI,2*M_PI);
        double dpsi2 = fmod((*state2)[5] - (*statePrev)[5]+4*M_PI,2*M_PI);
        double baseLineMin1 = lookupTable[(int)floor(N*a1/(2*M_PI))][(int)floor(N*dpsi1/(2*M_PI))]*FIXEDWING_SYSTEM::r_min;
        double baseLineMin2 = lookupTable[(int)floor(N*a2/(2*M_PI))][(int)floor(N*dpsi2/(2*M_PI))]*FIXEDWING_SYSTEM::r_min;
        if( BASE_LINE_FACTOR*sqrt( pow((*state1)[0] - g[0], 2.0) + pow((*state1)[1] - g[1], 2.0) ) < baseLineMin1 )
          c+= 0.045*DBL_MAX;
        if( BASE_LINE_FACTOR*sqrt( pow((*state2)[0] - g[0], 2.0) + pow((*state2)[1] - g[1], 2.0) ) < baseLineMin2 )
          c+= 0.045*DBL_MAX;
      }
      if(c<=costOrientation && this->isVisible(s))
      {
        g[3] = s[3];
        g[5] = s[5];
        costOrientation = c;
        orSolFound = true;
      }
    } 
    if(this->VPSolver->getObjVal()+xxCompensate+costOrientation<cost || (this->VPSolver->getObjVal()+xxCompensate+costOrientation==cost && (rand()%100)>=80))
    {
      best = g;
      cost = this->VPSolver->getObjVal()+xxCompensate+costOrientation;
    }
    solFound |= solFoundLocal;
  }
  if(!solFound)
  {
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
    if(this->initialized)
    {
      return (*statePrev);
    }
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
    if(obstacleCounter == maxPW)
    {
      std::string pkgPath = ros::package::getPath("koptplanner");
      std::fstream plannerLog;
      plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
      if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
      plannerLog << "-->Too may obstacles in the dual sampling space! Try increasing the maximum obstacle depth or adapt the scenario.\n";
      plannerLog << "   x1: (" << this->x1[0] << ", " << this->x1[1] << ", " << this->x1[2] << ")\n";
      plannerLog << "   x2: (" << this->x2[0] << ", " << this->x2[1] << ", " << this->x2[2] << ")\n";
      plannerLog << "   x3: (" << this->x3[0] << ", " << this->x3[1] << ", " << this->x3[2] << ")\n";
      plannerLog.close();
      koptError = OBSTACLE_INFEASIBILITY;
    }
  }
  if(!orSolFound)
  {
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
    if(this->initialized)
    {
      return (*statePrev);
    }
    else
    {
      std::string pkgPath = ros::package::getPath("koptplanner");
      std::fstream plannerLog;
      plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
      if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
      plannerLog << "-->No feasible heading to inspect triangle. Try increaing the minimal inspection distance or increase mesh resolution.\n";
      plannerLog << "   x1: (" << this->x1[0] << ", " << this->x1[1] << ", " << this->x1[2] << ")\n";
      plannerLog << "   x2: (" << this->x2[0] << ", " << this->x2[1] << ", " << this->x2[2] << ")\n";
      plannerLog << "   x3: (" << this->x3[0] << ", " << this->x3[1] << ", " << this->x3[2] << ")\n";
      plannerLog.close();
    }
    koptError = VIEWPOINT_INFEASIBILITY;
  }
  for(int i = 0; i<6; i++)
    assert(best[i]<1e9&& best[i]>-1e9);
#ifdef __TIMING_INFO__
  gettimeofday(&time, NULL);
  time_DBS += time.tv_sec * 1000000 + time.tv_usec;
#endif
  return best;
}

template<class System_t, class State_t, class Vector_t, class region_t>
double FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::AvoidObstacle(real_t* lb, real_t* ub, int Nobs, region_t* obs, double xxCompensate, StateVector& g)
{
  double opt = DBL_MAX;
  real_t xOpt[6];
  if(Nobs>g_max_obstacle_depth || ros::isShuttingDown())
  {
    return -1.0;
  }
  real_t lb_l[8] = {lb[0], lb[1], lb[2], lb[3], lb[4], lb[5], lb[6], lb[7]};
  real_t ub_l[8] = {ub[0], ub[1], ub[2], ub[3], ub[4], ub[5], ub[6], ub[7]};
  // -- min --
  for(int i = 0; i<3; i++)
  {
    int nWSR = 100;
    if(i = 2)
      lb_l[i] = std::max(obs->center[i]+obs->size[i]/2+g_security_distance, lb_l[i]);
    else
      lb_l[i] = std::max(obs->center[i]+obs->size[i]/2+g_security_distance+System_t::r_min, lb_l[i]);
    real_t lb_l2[8] = {lb_l[0], lb_l[1], lb_l[2], lb_l[3], lb_l[4], lb_l[5], lb_l[6], lb_l[7]};
    real_t ub_l2[8] = {ub_l[0], ub_l[1], ub_l[2], ub_l[3], ub_l[4], ub_l[5], ub_l[6], ub_l[7]};
    if(SUCCESSFUL_RETURN ==  this->VPSolver->hotstart( this->d,lb_l,ub_l,this->lbA,this->ubA, nWSR ))
    {
      if(this->VPSolver->getObjVal()+xxCompensate<opt)
      {
        if(RET_QP_NOT_SOLVED == this->VPSolver->getPrimalSolution( xOpt ))
          continue;
        StateVector gPot = g;
        for( int k = 0; k<8; k++)
        {
          lb_l[k] = lb[k];
          ub_l[k] = ub[k];
        }
        if(i = 2)
          lb_l[i] = std::max(obs->center[i]+obs->size[i]/2+g_security_distance, lb_l[i]);
        else
          lb_l[i] = std::max(obs->center[i]+obs->size[i]/2+g_security_distance+System_t::r_min, lb_l[i]);
        gPot[0] = xOpt[0]; gPot[1] = xOpt[1]; gPot[2] = xOpt[2];
        region_t* obsPot = NULL;
        if(obsPot = this->IsInCollision(gPot))
        {
          double optPot = DBL_MAX;
          if(opt>(optPot = this->AvoidObstacle(lb_l, ub_l, Nobs+1, obsPot, xxCompensate, gPot)))
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
    for( int k = 0; k<8; k++)
    {
      lb_l[k] = lb[k];
      ub_l[k] = ub[k];
    }
  }
  // -- max --
  for(int i = 0; i<3; i++)
  {
    int nWSR = 100;
    if(i = 2)
      ub_l[i] = std::min(obs->center[i]-obs->size[i]/2-g_security_distance, ub_l[i]);
    else
      ub_l[i] = std::min(obs->center[i]-obs->size[i]/2-g_security_distance-System_t::r_min, ub_l[i]);
    real_t lb_l2[8] = {lb_l[0], lb_l[1], lb_l[2], lb_l[3], lb_l[4], lb_l[5], lb_l[6], lb_l[7]};
    real_t ub_l2[8] = {ub_l[0], ub_l[1], ub_l[2], ub_l[3], ub_l[4], ub_l[5], ub_l[6], ub_l[7]};
    if(SUCCESSFUL_RETURN == this->VPSolver->hotstart( this->d,lb_l,ub_l,this->lbA,this->ubA, nWSR ))
    {
      if(this->VPSolver->getObjVal()+xxCompensate<opt)
      {
        if(RET_QP_NOT_SOLVED == this->VPSolver->getPrimalSolution( xOpt ))
          continue;
        StateVector gPot = g;
        for( int k = 0; k<8; k++)
        {
          lb_l[k] = lb[k];
          ub_l[k] = ub[k];
        }
        if(i = 2)
          ub_l[i] = std::min(obs->center[i]-obs->size[i]/2-g_security_distance, ub_l[i]);
        else
          ub_l[i] = std::min(obs->center[i]-obs->size[i]/2-g_security_distance-System_t::r_min, ub_l[i]);
        gPot[0] = xOpt[0]; gPot[1] = xOpt[1]; gPot[2] = xOpt[2];
        region_t* obsPot = NULL;
        if(obsPot = this->IsInCollision(gPot))
        {
          double optPot = DBL_MAX;
          if(opt>(optPot = this->AvoidObstacle(lb_l, ub_l, Nobs+1, obsPot, xxCompensate, gPot)))
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
    for( int k = 0; k<8; k++)
    {
      lb_l[k] = lb[k];
      ub_l[k] = ub[k];
    }
  }
  return opt;
}

template<class State_t>
FixedWing::Trajectory<State_t>::Trajectory ()
{

}


template<class State_t>
FixedWing::Trajectory<State_t>::~Trajectory ()
{

}


template<class State_t>
FixedWing::Trajectory<State_t>::Trajectory (const FixedWing::Trajectory<State_t> &trajectoryIn)
{
  this->endState = new State_t (trajectoryIn.getEndState()); 
}

template<class Trajectory_t, class State_t, class region_t>
FixedWing::System<Trajectory_t, State_t, region_t>::System ()
{
  this->numDimensions = 0;
}


template<class Trajectory_t, class State_t, class region_t>
FixedWing::System<Trajectory_t, State_t, region_t>::~System ()
{
    
}


template<class Trajectory_t, class State_t, class region_t>
int FixedWing::System<Trajectory_t, State_t, region_t>::sampleState (State_t &randomStateOut)
{
  randomStateOut.setNumDimensions (this->numDimensions);
  
  for (int i = 0; i < this->numDimensions; i++)
  {
    randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*this->regionOperating.size[i] 
    - this->regionOperating.size[i]/2.0 + this->regionOperating.center[i];
  }
  
  if (this->IsInCollision (randomStateOut.x))
      return 0;
  
  return 1;
}

template<class Trajectory_t, class State_t, class region_t>
int FixedWing::System<Trajectory_t, State_t, region_t>::extendTo (State_t &stateFromIn, State_t &stateTowardsIn, Trajectory_t &trajectoryOut, bool &exactConnectionOut)
{
  // run BVP Solver
  // for combinations of circle midpoints
  float minLength = FLT_MAX;
  float angle1 = 0;
  float angle2 = 0;
  float midPoint1XG = 0;
  float midPoint2XG = 0;
  float midPoint1YG = 0;
  float midPoint2YG = 0;
  float angle_con = 0;
  float left1;
  float left2;
  // ============================== left to left
  float length = FLT_MAX;
  float midPoint1X = stateFromIn[0]-r_min*sin(stateFromIn[5]);
  float midPoint1Y = stateFromIn[1]+r_min*cos(stateFromIn[5]);
  float midPoint2X = stateTowardsIn[0]-r_min*sin(stateTowardsIn[5]);
  float midPoint2Y = stateTowardsIn[1]+r_min*cos(stateTowardsIn[5]);
  float ang1 = stateFromIn[5];
  float ang2 = stateTowardsIn[5];
  if(ang1<0)
    ang1 += 2 * M_PI;
  if(ang2<0)
    ang2 += 2 * M_PI;
  if(midPoint1X == midPoint2X)
    midPoint1X += 1E-10; // to prevent division by zero
  float ang_midp = atan((midPoint2Y-midPoint1Y)/(midPoint2X-midPoint1X));
  if(midPoint1X>midPoint2X)
    ang_midp += M_PI;
  if(ang_midp<0)
    ang_midp += 2 * M_PI;
  if(ang1>ang_midp)
    ang1 -= 2 * M_PI;
  if(ang2<ang_midp)
    ang2 += 2 * M_PI;
  length = pow(SQ(midPoint2X-midPoint1X)+SQ(midPoint2Y-midPoint1Y),0.5) + r_min * (ang2 - ang1);

  // get total length
  if(length < minLength)
  {
    minLength = length;
    angle1 = ang1;
    angle2 = ang2;
    midPoint1XG = midPoint1X;
    midPoint2XG = midPoint2X;
    midPoint1YG = midPoint1Y;
    midPoint2YG = midPoint2Y;
    angle_con = ang_midp;
    left1 = 1;
    left2 = 1;
  }
  // =============================== right to right
  midPoint1X = stateFromIn[0]+r_min*sin(stateFromIn[5]);
  midPoint1Y = stateFromIn[1]-r_min*cos(stateFromIn[5]);
  midPoint2X = stateTowardsIn[0]+r_min*sin(stateTowardsIn[5]);
  midPoint2Y = stateTowardsIn[1]-r_min*cos(stateTowardsIn[5]);
  ang1 = stateFromIn[5];
  ang2 = stateTowardsIn[5];
  if(ang1<0)
    ang1 += 2 * M_PI;
  if(ang2<0)
    ang2 += 2 * M_PI;
  if(midPoint1X == midPoint2X)
    midPoint1X += 1E-10; // to prevent division by zero
  ang_midp = atan((midPoint2Y-midPoint1Y)/(midPoint2X-midPoint1X));
  if(midPoint1X>midPoint2X)
    ang_midp += M_PI;
  if(ang_midp<0)
    ang_midp += 2 * M_PI;
  if(ang1<ang_midp)
    ang1 += 2 * M_PI;
  if(ang2>ang_midp)
    ang2 -= 2 * M_PI;
  length = pow(SQ(midPoint2X-midPoint1X)+SQ(midPoint2Y-midPoint1Y),0.5) + r_min * (ang1 - ang2);

  // get total length
  if(length < minLength)
  {
    minLength = length;
    angle1 = ang1;
    angle2 = ang2;
    midPoint1XG = midPoint1X;
    midPoint2XG = midPoint2X;
    midPoint1YG = midPoint1Y;
    midPoint2YG = midPoint2Y;
    angle_con = ang_midp;
    left1 = -1;
    left2 = -1;
  }
  // =============================== left to right
  midPoint1X = stateFromIn[0]-r_min*sin(stateFromIn[5]);
  midPoint1Y = stateFromIn[1]+r_min*cos(stateFromIn[5]);
  midPoint2X = stateTowardsIn[0]+r_min*sin(stateTowardsIn[5]);
  midPoint2Y = stateTowardsIn[1]-r_min*cos(stateTowardsIn[5]);
  ang1 = stateFromIn[5];
  ang2 = stateTowardsIn[5];
  if(ang1<0)
    ang1 += 2 * M_PI;
  if(ang2<0)
    ang2 += 2 * M_PI;
  if(midPoint1X == midPoint2X)
    midPoint1X += 1E-10; // to prevent division by zero
  ang_midp = atan((midPoint2Y-midPoint1Y)/(midPoint2X-midPoint1X));
  if(midPoint1X>midPoint2X)
    ang_midp += M_PI;
  if(ang_midp<0)
    ang_midp += 2 * M_PI;
  length = pow(SQ(midPoint2X-midPoint1X)+SQ(midPoint2Y-midPoint1Y),0.5);
  float ang_con = asin(r_min/(0.5*length)) + ang_midp;
  if(ang_con> 2* M_PI)
    ang_con -= 2 * M_PI;
  if(ang_con<0)
    ang_con += 2 * M_PI;
  if(ang1>ang_con)
    ang1 -= 2 * M_PI;
  if(ang2>ang_con)
    ang2 -= 2 * M_PI;
  length = sqrt(SQ(length) - 4.0 * SQ(r_min)) + r_min * (2 * ang_con - ang1 - ang2);

  // get total length
  if(length < minLength)
  {
    minLength = length;
    angle1 = ang1;
    angle2 = ang2;
    midPoint1XG = midPoint1X;
    midPoint2XG = midPoint2X;
    midPoint1YG = midPoint1Y;
    midPoint2YG = midPoint2Y;
    angle_con = ang_con;
    left1 = 1;
    left2 = -1;
  }
  // =============================== right to left
  midPoint1X = stateFromIn[0]+r_min*sin(stateFromIn[5]);
  midPoint1Y = stateFromIn[1]-r_min*cos(stateFromIn[5]);
  midPoint2X = stateTowardsIn[0]-r_min*sin(stateTowardsIn[5]);
  midPoint2Y = stateTowardsIn[1]+r_min*cos(stateTowardsIn[5]);
  ang1 = stateFromIn[5];
  ang2 = stateTowardsIn[5];
  if(ang1<0)
    ang1 += 2 * M_PI;
  if(ang2<0)
    ang2 += 2 * M_PI;
  if(midPoint1X == midPoint2X)
    midPoint1X += 1E-10; // to prevent division by zero
  ang_midp = atan((midPoint2Y-midPoint1Y)/(midPoint2X-midPoint1X));
  if(midPoint1X>midPoint2X)
    ang_midp += M_PI;
  if(ang_midp<0)
    ang_midp += 2 * M_PI;
  length = pow(SQ(midPoint2X-midPoint1X)+SQ(midPoint2Y-midPoint1Y),0.5);
  ang_con = ang_midp - asin(r_min/(0.5*length));
  if(ang_con> 2* M_PI)
    ang_con -= 2 * M_PI;
  if(ang_con<0)
    ang_con += 2 * M_PI;
  if(ang1<ang_con)
    ang1 += 2 * M_PI;
  if(ang2<ang_con)
    ang2 += 2 * M_PI;
  length = sqrt(SQ(length) - 4.0 * SQ(r_min)) + r_min * (ang1 + ang2 - 2 * ang_con);

  // get total length
  if(length < minLength)
  {
    minLength = length;
    angle1 = ang1;
    angle2 = ang2;
    midPoint1XG = midPoint1X;
    midPoint2XG = midPoint2X;
    midPoint1YG = midPoint1Y;
    midPoint2YG = midPoint2Y;
    angle_con = ang_con;
    left1 = -1;
    left2 = 1;
  }
  // vertical displacement
  float dz = stateTowardsIn[2] - stateFromIn[2];
  double additionalCircles = 0.0;
  while(abs(dz/minLength) > g_maxClimbSinkRate)
  {
    minLength+=r_min*2.0*M_PI;
    additionalCircles+=1.0;
  }
  float d = stateFromIn[2];
  float c = stateFromIn[4];
  float b = 3*dz - 4*stateFromIn[4] - stateTowardsIn[4];
  float a = (stateTowardsIn[4] + stateFromIn[4])/3 - 2*b/3;
   
  // compute trajectory
  double deltaX = midPoint2XG+r_min*sin(angle_con)*left2-midPoint1XG-r_min*sin(angle_con)*left1;
  double deltaY = midPoint2YG-r_min*cos(angle_con)*left2-midPoint1YG+r_min*cos(angle_con)*left1;
  float abscissaResolution0 = g_discretization_step/std::max(r_min*abs(angle_con-angle1),(double)g_discretization_step);
  float abscissaResolution1 = g_discretization_step/std::max(sqrt(SQ(deltaX) + SQ(deltaY)),(double)g_discretization_step);
  float abscissaResolution2 = g_discretization_step/std::max(r_min*abs(angle2-angle_con),(double)g_discretization_step);
  float abscissaResolution3 = g_discretization_step/std::max(r_min*2*M_PI,(double)g_discretization_step);
  State_t sampledState; sampledState.setNumDimensions(this->numDimensions);
  Trajectory_t trajectoryTemp;
  trajectoryTemp.states_.clear();
  trajectoryTemp.states_.reserve(23);
  for (float t=0.0; t<=1; t+=abscissaResolution0)
  {
    sampledState[0] = midPoint1XG+r_min*sin(angle1+t*(angle_con-angle1))*left1;
    sampledState[1] = midPoint1YG-r_min*cos(angle1+t*(angle_con-angle1))*left1;
    sampledState[5] = angle1+t*(angle_con-angle1);
    if(sampledState[5] > M_PI)
      sampledState[5] -= 2 * M_PI;
    if(sampledState[5] < -M_PI)
      sampledState[5] += 2 * M_PI;
    sampledState[3] = 0;
    if(sampledState[3] > M_PI)
      sampledState[3] -= 2 * M_PI;
    if(sampledState[3] < -M_PI)
      sampledState[3] += 2 * M_PI;
    sampledState[2] = stateFromIn[2] + (dz/minLength)*r_min*t*abs(angle_con-angle1);
    sampledState[4] = 0;
    trajectoryTemp.states_.push_back(sampledState);
  }
  for (float t=0.0; t<=1.0; t+=abscissaResolution1)
  {
    sampledState[0] = midPoint1XG+r_min*sin(angle_con)*left1+t*(deltaX);
    sampledState[1] = midPoint1YG-r_min*cos(angle_con)*left1+t*(deltaY);
    sampledState[5] = angle_con;
    if(sampledState[5] > M_PI)
      sampledState[5] -= 2 * M_PI;
    if(sampledState[5] < -M_PI)
      sampledState[5] += 2 * M_PI;
    sampledState[3] = 0;
    if(sampledState[3] > M_PI)
      sampledState[3] -= 2 * M_PI;
    if(sampledState[3] < -M_PI)
      sampledState[3] += 2 * M_PI;
    sampledState[2] = stateFromIn[2] + (dz/minLength)*(r_min*abs(angle_con-angle1)+t*(sqrt(SQ(deltaX)+SQ(deltaY))));
    sampledState[4] = 0;
    trajectoryTemp.states_.push_back(sampledState);
  }
  for (float t=0.0; t<=1; t+=abscissaResolution2)
  {
    sampledState[0] = midPoint2XG+r_min*sin(angle_con+t*(angle2-angle_con))*left2;
    sampledState[1] = midPoint2YG-r_min*cos(angle_con+t*(angle2-angle_con))*left2;
    sampledState[5] = angle_con+t*(angle2-angle_con);
    if(sampledState[5] > M_PI)
      sampledState[5] -= 2 * M_PI;
    if(sampledState[5] < -M_PI)
      sampledState[5] += 2 * M_PI;
    sampledState[3] = 0;
    if(sampledState[3] > M_PI)
      sampledState[3] -= 2 * M_PI;
    if(sampledState[3] < -M_PI)
      sampledState[3] += 2 * M_PI;
    sampledState[2] = stateFromIn[2] + (dz/minLength)*(r_min*(abs(angle_con-angle1) + t*abs(angle2-angle_con) )+(sqrt(SQ(deltaX)+SQ(deltaY))));
    sampledState[4] = 0;
    trajectoryTemp.states_.push_back(sampledState);
  }
  // add additional circles to compensate for height difference
  for (float t=0.0001; t<=additionalCircles; t+=abscissaResolution3)
  {
    sampledState[0] = midPoint2XG+r_min*sin(angle2+t*2*M_PI*left2)*left2;
    sampledState[1] = midPoint2YG-r_min*cos(angle2+t*2*M_PI*left2)*left2;
    sampledState[5] = angle_con+t*(angle2-angle_con);
    if(sampledState[5] > M_PI)
      sampledState[5] -= 2 * M_PI;
    if(sampledState[5] < -M_PI)
      sampledState[5] += 2 * M_PI;
    sampledState[3] = 0;
    if(sampledState[3] > M_PI)
      sampledState[3] -= 2 * M_PI;
    if(sampledState[3] < -M_PI)
      sampledState[3] += 2 * M_PI;
    sampledState[2] = stateFromIn[2] + (dz/minLength)*(r_min*(t*2*M_PI + abs(angle_con-angle1) + abs(angle2-angle_con))+(sqrt(SQ(deltaX)+SQ(deltaY))));
    sampledState[4] = 0;
    trajectoryTemp.states_.push_back(sampledState);
  }
  minLength = sqrt(SQ(fabs(dz)) + SQ(minLength));
  // check for collision and state space boundaries
  for(typename std::vector<State>::iterator it = trajectoryTemp.states_.begin(); it != trajectoryTemp.states_.end(); it++)
  {
    double * tmp = new double[DIMENSIONALITY];
    for(int i = 0; i<DIMENSIONALITY; i++)
      tmp[i] = (*it)[i];
    if(this->IsInCollision(tmp))
    {
      trajectoryOut.totalVariation = FLT_MAX;
      return 0;
    }
    delete[] tmp;
    
  }

  trajectoryOut.states_.clear();
  trajectoryOut.states_ = trajectoryTemp.states_;
  trajectoryOut.endState = new State (stateTowardsIn);
  trajectoryOut.totalVariation = minLength;
  assert(minLength>=0);
  exactConnectionOut = true;
    
  return 1;
}

template<class Trajectory_t, class State_t, class region_t>
double FixedWing::System<Trajectory_t, State_t, region_t>::evaluateExtensionCost (State_t& stateFromIn, State_t& stateTowardsIn, bool &exactConnectionOut)
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
void FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::setCamBoundNormals()
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
Vector3f FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::camBoundRotated(Vector3f normal, double roll, double yaw)
{
  Vector3f x(1, 0, 0);
  Vector3f z(0, 0, 1);
  AngleAxisf mroll = AngleAxisf(roll, x);
  AngleAxisf myaw = AngleAxisf(yaw, z);
  return myaw*(mroll*normal);
}

#endif
