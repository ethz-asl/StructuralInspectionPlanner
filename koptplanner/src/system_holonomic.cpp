/*!
 * \file system_holonomic.cpp
 *
 * More elaborate description
 */
#ifndef __SYSTEM_HOLONOMIC_CPP__
#define __SYSTEM_HOLONOMIC_CPP__

#include <ros/ros.h>
#include <cmath>
#include <cstdlib>

#include <iostream>

#include "planner_rrts/system_holonomic.h"
#include "planner_rrts/system_holonomic.hpp"

using namespace std;
using namespace Holonomic;

region::region ()
{
  numDimensions = 0;
  
  center = NULL;
  size = NULL;
}


region::~region ()
{
  if (center)
    delete [] center;
  if (size)
    delete [] size;
}


int region::setNumDimensions (int numDimensionsIn)
{
  numDimensions = numDimensionsIn;
  
  if (center)
    delete [] center;
  center = new double[numDimensions];
  
  if (size)
    delete [] size;
  size = new double[numDimensions];
  
  return 1;
}


State::State ()
{
  numDimensions = 0;
  
  x = NULL;
}


State::~State ()
{
  if (x)
    delete [] x;
}


State::State (const State &stateIn)
{
  numDimensions = stateIn.numDimensions;
  
  if (numDimensions > 0)
  {
    x = new double[numDimensions];
    
    for (int i = 0; i < numDimensions; i++) 
      x[i] = stateIn.x[i];
  }
  else
  {
    x = NULL;
  }
}


State& State::operator=(const State &stateIn)
{
  if (this == &stateIn)
    return *this;
  
  if (numDimensions != stateIn.numDimensions)
  {
    if (x) 
      delete [] x;
    numDimensions = stateIn.numDimensions;
    if (numDimensions > 0)
      x = new double[numDimensions];
  }
  
  for (int i = 0; i < numDimensions; i++) 
    x[i] = stateIn.x[i];
  
  return *this;
}


int State::setNumDimensions (int numDimensionsIn)
{
  if (x)
    delete [] x;
  
  if (numDimensions < 0)
    return 0;
  
  numDimensions = numDimensionsIn;
  
  if (numDimensions > 0)
    x = new double[numDimensions];
  
  return 1;
}

#endif
