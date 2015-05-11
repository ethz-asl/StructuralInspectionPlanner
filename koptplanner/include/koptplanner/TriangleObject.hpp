/*!
 * \file TriangleObject.hpp
 *
 * More elaborate description
 */
#ifndef __TRIANGLE_OBJECT_HPP__
#define __TRIANGLE_OBJECT_HPP__

#include "ros/ros.h"
#include <list>
#include "float.h"
#include <Eigen/Geometry>
#include "koptplanner/TriangleObject.h"
#include "koptplanner/plan.hpp"
#include "optec/qpOASES.hpp"
#include "koptplanner/ptpPlanner.hpp"
#include <octomap/octomap.h>

using namespace Holonomic;
USING_NAMESPACE_QPOASES

#define SQ(x) ((x)*(x))
#define OBSTACLE_DIST_THRESHOLD Triangle::maxDist

#ifdef __USE_SINGLE_PRECISION__
 #define REAL_T_MAX_VAL 1.0e9
#else
 #define REAL_T_MAX_VAL 1.0e9
#endif

#define X_MIN (problemBoundary.center[0]-problemBoundary.size[0]/2)
#define Y_MIN (problemBoundary.center[1]-problemBoundary.size[1]/2)
#define Z_MIN (problemBoundary.center[2]-problemBoundary.size[2]/2)
#define X_MAX (problemBoundary.center[0]+problemBoundary.size[0]/2)
#define Y_MAX (problemBoundary.center[1]+problemBoundary.size[1]/2)
#define Z_MAX (problemBoundary.center[2]+problemBoundary.size[2]/2)

#ifdef __TIMING_INFO__
 extern long time_DBS;
#endif
extern MDL_NAMESPACE::region problemBoundary;
extern double g_discretization_step;
extern double g_security_distance;
extern double g_max_obs_dim;


template<class System_t, class State_t, class Vector_t, class region_t>
Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::Triangle()
{
  Fixpoint = false;
  VPSolver = new QProblem( 3,4 );
  H = new real_t[3*3];
  A = new real_t[4*3];
  d = new real_t[3];
  lbA = new real_t[4];
  ubA = new real_t[4];
}

template<class System_t, class State_t, class Vector_t, class region_t>
Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::~Triangle()
{
  delete VPSolver;
  delete[] H;
  delete[] A;
  delete[] d;
  delete[] lbA;
  delete[] ubA;
}

template<class System_t, class State_t, class Vector_t, class region_t>
void Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::setParam(double incidenceAngleIn, double minDistIn, double maxDistIn)
{
  incidenceAngle = incidenceAngleIn;
  minDist = minDistIn;
  maxDist = maxDistIn;
}

template<class System_t, class State_t, class Vector_t, class region_t>
bool Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::isVisible(State_t state)
{
  Vector_t s;
  for(int i = 0; i<DIMENSIONALITY; i++)
    s[i] = state[i];
  return this->isVisible(s);
}

template<class System_t, class State_t, class Vector_t, class region_t>
bool Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::isVisible(Vector_t s)
{
  if(aabs.dot(s-x1-aabs*minDist)<0)
    return false;
  if(aabs.dot(s-x1-aabs*maxDist)>0)
    return false;
  if((s-x1).dot(n1)<0)
    return false;
  if((s-x2).dot(n2)<0)
    return false;
  if((s-x3).dot(n3)<0)
    return false;
  return !IsInCollision(s);
}

template<class System_t, class State_t, class Vector_t, class region_t>
region_t* Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::IsInCollision (Vector_t &stateIn)
{
  double rmin = 0;
#ifdef USE_FIXEDWING_MODEL
  rmin = System_t::r_min;
#endif
  // obstacle check
  for (typename std::list<region_t*>::iterator iter = System_t::obstacles.begin(); iter != System_t::obstacles.end(); iter++)
  {
    if (fabs((*iter)->center[0] - stateIn[0]) < (*iter)->size[0]/2.0 + g_security_distance + rmin &&
        fabs((*iter)->center[1] - stateIn[1]) < (*iter)->size[1]/2.0 + g_security_distance + rmin &&
        fabs((*iter)->center[2] - stateIn[2]) < (*iter)->size[2]/2.0 + g_security_distance)
    {
      return *iter;
    }
  }
  
  // ray shooting
  for (typename std::list<region_t*>::iterator iter = System_t::obstacles.begin(); iter != System_t::obstacles.end(); iter++)
  {
    if (fabs((*iter)->center[0] - stateIn[0]) > OBSTACLE_DIST_THRESHOLD + g_max_obs_dim ||
        fabs((*iter)->center[1] - stateIn[1]) > OBSTACLE_DIST_THRESHOLD + g_max_obs_dim ||
        fabs((*iter)->center[2] - stateIn[2]) > OBSTACLE_DIST_THRESHOLD + g_max_obs_dim ||
        (*iter)->occupied>0)
    {
      continue;
    }
    float f_disc = g_discretization_step / sqrt(SQ(x1[0] - stateIn[0]) + SQ(x1[1] - stateIn[1]) + SQ(x1[2] - stateIn[2]));
    for(float f = 0.0; f < 1.0; f+=f_disc)
    {
      if (fabs((*iter)->center[0] - (stateIn[0]*f+(1.0-f)*x1[0])) < (*iter)->size[0]/2.0 &&
          fabs((*iter)->center[1] - (stateIn[1]*f+(1.0-f)*x1[1])) < (*iter)->size[1]/2.0 &&
          fabs((*iter)->center[2] - (stateIn[2]*f+(1.0-f)*x1[2])) < (*iter)->size[2]/2.0)
      {
        return *iter;
      }
    }
    f_disc = g_discretization_step / sqrt(SQ(x2[0] - stateIn[0]) + SQ(x2[1] - stateIn[1]) + SQ(x2[2] - stateIn[2]));
    for(float f = 0.0; f < 1.0; f+=f_disc)
    {
      if (fabs((*iter)->center[0] - (stateIn[0]*f+(1.0-f)*x2[0])) < (*iter)->size[0]/2.0 &&
          fabs((*iter)->center[1] - (stateIn[1]*f+(1.0-f)*x2[1])) < (*iter)->size[1]/2.0 &&
          fabs((*iter)->center[2] - (stateIn[2]*f+(1.0-f)*x2[2])) < (*iter)->size[2]/2.0)
      {
        return *iter;
      }
    }
    f_disc = g_discretization_step / sqrt(SQ(x3[0] - stateIn[0]) + SQ(x3[1] - stateIn[1]) + SQ(x3[2] - stateIn[2]));
    for(float f = 0.0; f < 1.0; f+=f_disc)
    {
      if (fabs((*iter)->center[0] - (stateIn[0]*f+(1.0-f)*x3[0])) < (*iter)->size[0]/2.0 &&
          fabs((*iter)->center[1] - (stateIn[1]*f+(1.0-f)*x3[1])) < (*iter)->size[1]/2.0 &&
          fabs((*iter)->center[2] - (stateIn[2]*f+(1.0-f)*x3[2])) < (*iter)->size[2]/2.0)
      {
        return *iter;
      }
    }
  }
  return NULL;
} 


#endif
