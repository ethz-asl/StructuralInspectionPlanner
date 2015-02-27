/*!
 * \file Rotorcraft.cpp
 *
 * More elaborate description
 */
#ifndef __ROTORCRAFT_CPP__
#define __ROTORCRAFT_CPP__

#include "ros/ros.h"
#include "koptplanner/Rotorcraft.hpp"
#include "float.h"
#include <Eigen/Geometry>
#include "optec/qpOASES.hpp"

Rotorcraft::State::State ()
{

} 

Rotorcraft::State::~State ()
{

}

Rotorcraft::State::State (const Rotorcraft::State &stateIn)
{
  this->numDimensions = stateIn.numDimensions;
  
  if (this->numDimensions > 0) {
    x = new double[this->numDimensions];
    
    for (int i = 0; i < this->numDimensions; i++) 
      this->x[i] = stateIn.x[i];
  }
  else {
    this->x = NULL;
  }
}

#endif
