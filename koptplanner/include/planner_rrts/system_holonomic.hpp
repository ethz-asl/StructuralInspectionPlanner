/*!
 * \file system_holonomic.hpp
 *
 * More elaborate description
 */
#ifndef __SYSTEM_HOLONOMIC_HPP__
#define __SYSTEM_HOLONOMIC_HPP__

#include <ros/ros.h>
#include <cmath>
#include <cstdlib>

#include <iostream>

#include "planner_rrts/system_holonomic.h"
#include "koptplanner/plan.hpp"

extern double g_discretization_step;
extern double g_security_distance;

using namespace std;

template<class State_t>
Holonomic::Trajectory<State_t>::Trajectory ()
{
  endState = NULL;
  totalVariation = 0;
}


template<class State_t>
Holonomic::Trajectory<State_t>::~Trajectory ()
{
  if (endState)
    delete endState;
}


template<class State_t>
Holonomic::Trajectory<State_t>::Trajectory (const Trajectory<State_t> &trajectoryIn)
{
  endState = new State_t (trajectoryIn.getEndState()); 
}


template<class State_t>
Holonomic::Trajectory<State_t>& Holonomic::Trajectory<State_t>::operator=(const Holonomic::Trajectory<State_t> &trajectoryIn)
{
  if (this == &trajectoryIn)
    return *this;
  
  if (endState)
    delete endState;
  
  
  endState = new State_t (trajectoryIn.getEndState());
  
  totalVariation = trajectoryIn.totalVariation;
  
  return *this;
}


template<class State_t>
double Holonomic::Trajectory<State_t>::evaluateCost ()
{
  return totalVariation;
}


template<class Trajectory_t, class State_t, class region_t>
Holonomic::System<Trajectory_t, State_t, region_t>::System ()
{
  numDimensions = 0;
}


template<class Trajectory_t, class State_t, class region_t>
Holonomic::System<Trajectory_t, State_t, region_t>::~System ()
{
    
}


template<class Trajectory_t, class State_t, class region_t>
int Holonomic::System<Trajectory_t, State_t, region_t>::setNumDimensions (int numDimensionsIn)
{
  if (numDimensions < 0)
    return 0;
  
  numDimensions = numDimensionsIn;
  
  rootState.setNumDimensions (numDimensions);
  
  return 1;
}


template<class Trajectory_t, class State_t, class region_t>
int Holonomic::System<Trajectory_t, State_t, region_t>::getStateKey (State_t& stateIn, double* stateKey)
{
  for (int i = 0; i < numDimensions; i++)
  {
    stateKey[i] =  stateIn.x[i] / regionOperating.size[i];
  }
  
  return 1;
}



template<class Trajectory_t, class State_t, class region_t>
bool Holonomic::System<Trajectory_t, State_t, region_t>::isReachingTarget (State_t &stateIn)
{
  for (int i = 0; i < numDimensions; i++)
  {
    if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/2.0) 
      return false;
  }
  
  return true;
}


template<class Trajectory_t, class State_t, class region_t>
bool Holonomic::System<Trajectory_t, State_t, region_t>::IsInCollision (double *stateIn)
{
  for (typename list<region_t*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++)
  {
    if(fabs((*iter)->center[0] - stateIn[0]) < (*iter)->size[0]/2.0 + g_security_distance &&
       fabs((*iter)->center[1] - stateIn[1]) < (*iter)->size[1]/2.0 + g_security_distance &&
       fabs((*iter)->center[2] - stateIn[2]) < (*iter)->size[2]/2.0 + g_security_distance)
    {
      return true;
    }
  }

  return false;
}

template<class Trajectory_t, class State_t, class region_t>
int Holonomic::System<Trajectory_t, State_t, region_t>::sampleState (State_t &randomStateOut)
{
  randomStateOut.setNumDimensions (numDimensions);
  
  for (int i = 0; i < numDimensions; i++)
  {
    randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i] 
    - regionOperating.size[i]/2.0 + regionOperating.center[i];
  }
  
  if (IsInCollision (randomStateOut.x))
    return 0;
  
  return 1;
}



template<class Trajectory_t, class State_t, class region_t>
int Holonomic::System<Trajectory_t, State_t, region_t>::extendTo (State_t &stateFromIn, State_t &stateTowardsIn, Trajectory_t &trajectoryOut, bool &exactConnectionOut)
{
  double *dists = new double[numDimensions];
  for (int i = 0; i < numDimensions; i++) 
    dists[i] = stateTowardsIn.x[i] - stateFromIn.x[i];
  
  double distTotal = 0.0;
  for (int i = 0; i < numDimensions; i++) 
    distTotal += dists[i]*dists[i];
  distTotal = sqrt (distTotal);
  
  double incrementTotal = distTotal/g_discretization_step;
  // normalize the distance according to the disretization step
  for (int i = 0; i < numDimensions; i++)
    dists[i] /= incrementTotal;
  
  int numSegments = (int)floor(incrementTotal);
  
  double *stateCurr = new double[numDimensions];
  for (int i = 0; i < numDimensions; i++) 
    stateCurr[i] = stateFromIn.x[i];
  
  for (int i = 0; i < numSegments; i++)
  {
    if (IsInCollision (stateCurr))  
      return 0;

    for (int i = 0; i < numDimensions; i++)
      stateCurr[i] += dists[i];
  }
  
  if (IsInCollision (stateTowardsIn.x))
    return 0;
  
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
double Holonomic::System<Trajectory_t, State_t, region_t>::evaluateExtensionCost (State_t& stateFromIn, State_t& stateTowardsIn, bool &exactConnectionOut)
{
  exactConnectionOut = true;
  
  double distTotal = 0.0;
  for (int i = 0; i < numDimensions; i++)
  {
    double distCurr = stateTowardsIn.x[i] - stateFromIn.x[i];
    distTotal += distCurr*distCurr;
  }
  
  return sqrt(distTotal);
}


template<class Trajectory_t, class State_t, class region_t>
int Holonomic::System<Trajectory_t, State_t, region_t>::getTrajectory (State_t& stateFromIn, State_t& stateToIn, list<double*>& trajectoryOut)
{
  bool exact = false;

  Trajectory_t trajectoryO;
  this->extendTo(stateFromIn, stateToIn, trajectoryO, exact);
  for(typename std::vector<State_t>::iterator it = trajectoryO.states_.begin(); it != trajectoryO.states_.end(); it++)
  {
    double * tmp = new double[DIMENSIONALITY];
    for(int i = 0; i<DIMENSIONALITY; i++)
    {
      tmp[i] = (*it)[i];
    }
    trajectoryOut.push_back(tmp);
  }
  return 1;
}


template<class Trajectory_t, class State_t, class region_t>
double Holonomic::System<Trajectory_t, State_t, region_t>::evaluateCostToGo (State_t& stateIn)
{
  return 0;
}

#endif
