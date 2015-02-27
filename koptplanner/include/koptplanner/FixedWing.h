/*!
 * \file FixedWing.h
 *
 * More elaborate description
 */
#ifndef __FIXEDWING_H__
#define __FIXEDWING_H__

#include "ros/ros.h"
#include "koptplanner/plan.hpp"
#include "koptplanner/Rotorcraft.h"
#include "float.h"

#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

/*!
 * \brief FixedWing namespace
 *
 * More elaborate description
 */
namespace FixedWing
{
  class State;

  template<class State_t>
  class Trajectory;

  template<class Trajectory_t, class State_t, class region_t>
  class System;

  /*!
   * \brief Triangle class
   *
   * More elaborate description
   */
  template<class System_t, class State_t, class Vector_t, class region_t>
  class Triangle : public Rotorcraft::Triangle<System_t, State_t, Vector_t, region_t>
  {
  private:

    /*!
     * \brief Vector of precomputed camera bound normals for visibility check
     *
     * More elaborate description
     */
    static std::vector<Vector3f> camBoundNormal;
  public:

    /*!
     * \brief Triangle constructor
     *
     * More elaborate description
     */
    Triangle();

    /*!
     * \brief Initializes the constant part of the optimization problem
     *
     * More elaborate description
     */
    void init();
        
    /*!
     * \brief Checks the visibility of the object from state s
     *
     * More elaborate description
     * 
     * \param s State
     */
    bool isVisible(Vector_t s);
        
    /*!
     * \brief Viewpoint sampling function
     *
     * More elaborate description
     * 
     * \param state1 Preceding state
     * \param state2 Succeeding state
     * \param statePrev Previous state
     * 
     * \return Newly sampled viewpoint state
     */
    Vector_t dualBarrierSampling(Vector_t* state1, Vector_t* state2, Vector_t* statePrev);
        
    /*!
     * \brief Support function for viewpoint sampling
     *
     * If an obstacle is encountered while sampling the new viewpoint, its
     * neighbourhood is recursively searched maintaining a convex problem.
     * 
     * \param lb Lower bound vector
     * \param ub Upper bound vector
     * \param Nobs Number of obstacles, level of the recursive search
     * \param obs Reference to the encountered obstacle
     * \param xxCompensate Constant term of the optimization objective
     * \param g Reference of the solution vector
     * 
     * \return Cost of the solution, DBL_MAX if no solution is found
     */
    double AvoidObstacle(real_t* lb, real_t* ub, int Nobs, region_t* obs, double xxCompensate, StateVector& g);
        
    /*!
     * \brief Rotate a field o view boundary normal
     *
     * More elaborate description
     * 
     * \param normal Normal to rotate
     * \param roll Roll angle
     * \param yaw Yaw angle
     * 
     * \return Rotated normal vector
     */
    Vector3f camBoundRotated(Vector3f normal, double roll, double yaw);
        
    /*!
     * \brief Static function to precompute the field of view boundary normals
     *
     * More elaborate description
     */
    static void setCamBoundNormals();
  };

  /*!
   * \brief region class
   *
   * More elaborate description
   */
  class region : public Rotorcraft::region
  {

  };

  /*!
   * \brief State class
   *
   * More elaborate description
   */
  class State : public Rotorcraft::State
  {
  public:

    /*!
     * \brief State constructor
     *
     * More elaborate description
     */
    State ();

    /*!
     * \brief State destructor
     *
     * More elaborate description
     */
    ~State ();

    /*!
     * \brief State copy constructor
     *
     * More elaborate description
     */
    State (const State& stateIn);

    template<class Trajectory_t, class State_t, class region_t>
    friend class System;
    template<class State_t>
    friend class Trajectory;
  };

  /*!
   * \brief Trajectory class
   *
   * More elaborate description
   */
  template<class State_t>
  class Trajectory : public Rotorcraft::Trajectory<State_t>
  {
  public:

    /*!
     * \brief Trajectory constructor
     *
     * More elaborate description
     */
    Trajectory ();

    /*!
     * \brief Trajectory destructor
     *
     * More elaborate description
     */
    ~Trajectory ();

    /*!
     * \brief Trajectory copy constructor
     *
     * More elaborate description
     */
    Trajectory (const Trajectory<State_t>& trajectoryIn);

    template<class Trajectory_t, class State_t2, class region_t>
    friend class System;
    template<class System_t, class State_t2, class Vector_t, class region_t>
    friend class Triangle;
  };

  /*!
   * \brief System class
   *
   * More elaborate description
   */
  template<class Trajectory_t, class State_t, class region_t>
  class System : public Rotorcraft::System<Trajectory_t, State_t, region_t>
  {
  public:
    typedef Trajectory_t traj_c;

    /*!
     * \brief System constructor
     *
     * More elaborate description
     */
    System ();

    /*!
     * \brief System destructor
     *
     * More elaborate description
     */
    ~System ();
        
    /*!
     * \brief Boundary value solver
     *
     * More elaborate description
     * 
     * \param stateFromIn State to begin
     * \param stateTowardsIn State to end
     * \param trajectoryOut Trajectory object to store the computed trajectory
     * \param exactConnectionOut Bool to specify whether an exact connection has been established
     * 
     * \return 1 for success, 0 otherwise
     */
    virtual int extendTo (State_t &stateFromIn, State_t &stateTowardsIn, Trajectory_t &trajectoryOut, bool &exactConnectionOut);
        
    /*!
     * \brief Connection cost estimation
     *
     * More elaborate description
     * 
     * \param stateFromIn State to begin
     * \param stateTowardsIn State to end
     * \param exactConnectionOut Bool to specify whether an exact connection has been established
     * 
     * \return Cost for connection
     */
    double evaluateExtensionCost (State_t& stateFromIn, State_t& stateTowardsIn, bool &exactConnectionOut);
        
    /*!
     * \brief Sample a new state
     *
     * More elaborate description
     * 
     * \param randomStateOut Reference to state vector to store the new state in
     * 
     * \return 1 for success, 0 otherwise
     */
    int sampleState (State_t &randomStateOut);

    /*!
     * \brief Minimum turn radius
     *
     * More elaborate description
     */
    static double r_min;
  };
}

template<class System_t, class State_t, class Vector_t, class region_t>
std::vector<Vector3f> FixedWing::Triangle<System_t, State_t, Vector_t, region_t>::camBoundNormal;

#endif /* __FIXEDWING_H__ */
