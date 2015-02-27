/*!
 * \file FixedWing.cpp
 *
 * More elaborate description
 */
#ifndef __FIXEDWING_CPP__
#define __FIXEDWING_CPP__

#include "ros/ros.h"
#include "koptplanner/FixedWing.h"
#include "koptplanner/FixedWing.hpp"
#include "float.h"
#include <Eigen/Geometry>
#include "optec/qpOASES.hpp"

FixedWing::State::State ()
{

}

FixedWing::State::~State ()
{

}


FixedWing::State::State (const FixedWing::State &stateIn)
{
  this->numDimensions = stateIn.numDimensions;
  
  if (this->numDimensions > 0)
  {
    x = new double[this->numDimensions];
    
    for (int i = 0; i < this->numDimensions; i++) 
      this->x[i] = stateIn.x[i];
  }
  else
  {
    this->x = NULL;
  }
}

#endif
