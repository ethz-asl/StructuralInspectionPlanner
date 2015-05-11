/*!
 * \file plan.hpp
 *
 * More elaborate description
 */
#ifndef _PLAN_HPP_
#define _PLAN_HPP_

#include <eigen3/Eigen/Dense>
// To chose the used model (rotorcraft or fixed-wing) make
// the corresponding definition. Define only one model.
#define USE_ROTORCRAFT_MODEL
//#define USE_FIXEDWING_MODEL

// define to display time consumption of different
// parts of the algorithm
#define __TIMING_INFO__

////////////////////////////////////////////////////////

#define ANGABS(x) (fabs(x)>M_PI?2*M_PI-fabs(x):fabs(x))
#define LOOKUPTABLE_SIZE 50

#ifdef USE_HOLONOMIC_MODEL
 typedef Eigen::Vector3f StateVector;
#elif defined USE_ROTORCRAFT_MODEL
 typedef Eigen::Vector4f StateVector;
#elif defined USE_FIXEDWING_MODEL
 typedef Eigen::Matrix< float , 6 , 1> StateVector;
#endif

#if defined USE_ROTORCRAFT_MODEL && defined USE_FIXEDWING_MODEL
 exactly_one_model_can_be_defined;
#endif

#ifdef USE_ROTORCRAFT_MODEL
 #define USE_MODEL_NAMESPACE using namespace Rotorcraft;
 #define MDL_NAMESPACE Rotorcraft
 #define DIMENSIONALITY 4
#endif

#ifdef USE_FIXEDWING_MODEL
 #define USE_MODEL_NAMESPACE using namespace FixedWing;
 #define MDL_NAMESPACE FixedWing
 #define DIMENSIONALITY 6
 #define __ATSP__
#endif

enum koptError_t {
  SUCCESSFUL = 0,               // no error occured, successful execution
  OBSTACLE_INFEASIBILITY,       // the viewpoint is not feasible due to too many obstacles in the sampling region
  CONNECTION_INFEASIBILITY,     // no path could be found due to infeasible conection of viewpoints
  VIEWPOINT_INFEASIBILITY,      // the viewpoint sampling procedure could not find a valid viewpoint
  MISSING_PARAMETER,            // parameter is missing. not loaded?
  NO_START_POSITION,            // no starting position has been defined
  TOO_FEW_INSPECTION_AREAS,     // too few inspection areas have been defined in the service call
  INVALID_OBSTACLE_SHAPE,       // the required obstacle type is not supported
};

#endif
