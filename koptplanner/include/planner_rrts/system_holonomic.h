/*! 
 * \file system_holonomic.h
 *
 * More elaborate description
 */
#ifndef __RRTS_SYSTEM_HOLONOMIC_H__
#define __RRTS_SYSTEM_HOLONOMIC_H__

#include "koptplanner/plan.hpp"

#include <list>

namespace Holonomic
{

  template<class State_t>
  class Trajectory;

  template<class Trajectory_t, class State_t, class region_t>
  class System;
  
  /*!
   * \brief region class
   *
   * More elaborate description
   */
  class region
  {
  protected:
    
    int numDimensions;
    
  public:    
    
    /*!
     * \brief Cartesian coordinates of the center of the region
     *
     * More elaborate description
     */
    double *center;
    
    /*!
     * \brief Size of the region in cartesian coordinates
     *
     * More elaborate description
     */
    double *size;

    /*!
     * \brief Occupancy int, 0 is occupied
     *
     * More elaborate description
     */
    int occupied;
    
    /*!
     * \brief region constructor
     *
     * More elaborate description
     */
    region ();
    
    /*!
     * \brief region destructor
     *
     * More elaborate description
     */
    ~region ();
    
    /*!
     * \brief Sets the dimensionality of the region
     *
     * More elaborate description
     *
     * \param numDimensionsIn New number of dimensions.
     *
     */
    int setNumDimensions (int numDimensionsIn);
  };
  

  
  /*!
   * \brief State Class.
   *
   * A more elaborate description of the State class
   */
  class State
  {
  protected:
    
    int numDimensions;
    double *x;
    
  public:
    int setNumDimensions (int numDimensions);
    
    /*!
     * \brief State constructor
     *
     * More elaborate description
     */
    State ();
    
    /*!
     * \brief State desctructor
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
    
    /*!
     * \brief State assignment operator
     *
     * More elaborate description
     */
    State& operator= (const State& stateIn);
    
    /*!
     * \brief State bracket operator
     *
     * More elaborate description
     */
    double& operator[] (const int i) {return x[i];}
    
    template<class Trajectory_t, class State_t, class region_t>
    friend class System;
    template<class State_t>
    friend class Trajectory;
  };
  
  
  
  /*!
   * \brief Trajectory Class.
   *
   * A more elaborate description of the State class
   */
  template<class State_t>
  class Trajectory
  {
  protected:
    
    State_t *endState; 
    double totalVariation;  
    std::vector<State_t> states_;
    
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
     *
     * \param trajectoryIn The trajectory to be copied.
     *
     */
    Trajectory (const Trajectory<State_t>& trajectoryIn);
    
    /*!
     * \brief Trajectory assignment constructor
     *
     * More elaborate description
     *
     * \param trajectoryIn the trajectory to be copied.
     *
     */
    Trajectory& operator= (const Trajectory<State_t>& trajectoryIn);
    
    /*!
     * \brief Returns a reference to the end state of this trajectory.
     *
     * More elaborate description
     */
    State_t& getEndState () {return *endState;}
    
    /*!
     * \brief Returns a reference to the end state of this trajectory (constant).
     *
     * More elaborate description
     */
    State_t& getEndState () const {return *endState;}
    
    /*!
     * \brief Returns the cost of this trajectory.
     *
     * More elaborate description
     */
    double evaluateCost ();
    
    template<class Trajectory_t, class State_t2, class region_t>
    friend class System;
  };
  
  
  
  /*!
   * \brief System Class.
   *
   * A more elaborate description of the State class
   */
  template<class Trajectory_t, class State_t, class region_t>
  class System
  {
  protected:
    
    int numDimensions;
    
    State_t rootState;
    
  public:    
    bool IsInCollision (double *stateIn);

    /*!
     * \brief The operating region
     *
     * More elaborate description
     */
    region_t regionOperating;
    
    /*!
     * \brief The goal region
     *
     * More elaborate description
     */
    region_t regionGoal;
    
    /*!
     * \brief The list of all obstacles
     *
     * More elaborate description
     */
    static std::list<region_t*> obstacles;
    
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
    
    int setNumDimensions (int numDimensionsIn);
    
    /*!
     * \brief Returns the dimensionality of the Euclidean space.
     *
     * A more elaborate description.
     */
    int getNumDimensions () {return numDimensions;}
    
    /*!
     * \brief Returns a reference to the root state.
     *
     * A more elaborate description.
     */
    State_t& getRootState () {return rootState;}
    
    /*!
     * \brief Returns the statekey for the given state.
     *
     * A more elaborate description.
     *
     * \param stateIn the given state
     * \param stateKey the key to the state. An array of dimension getNumDimensions()
     *
     */
    int getStateKey (State_t &stateIn, double *stateKey);
    
    /*!
     * \brief Returns true of the given state reaches the target.
     *
     * A more elaborate description.
     */
    bool isReachingTarget (State_t &stateIn);
    
    /*!
     * \brief Returns a sample state.
     *
     * A more elaborate description.
     *
     * \param randomStateOut
     *
     */
    int sampleState (State_t &randomStateOut); 

    
    /*!
     * \brief Returns a the cost of the trajectory that connects stateFromIn and
     *        stateTowardsIn. The trajectory is also returned in trajectoryOut.
     *
     * A more elaborate description.
     * 
     * \param stateFromIn Initial state
     * \param stateTowardsIn Final state
     * \param trajectoryOut Trajectory that starts the from the initial state and 
     *                      reaches near the final state.
     * \param exactConnectionOut Set to true if the initial and the final states
     *                           can be connected exactly.
     *
     */
    virtual int extendTo (State_t &stateFromIn, State_t &stateTowardsIn, 
                  Trajectory_t &trajectoryOut, bool &exactConnectionOut) = 0; 
    
    /*!
     * \brief Returns the cost of the trajectory that connects stateFromIn and StateTowardsIn.
     *
     * A more elaborate description.
     *
     * \param stateFromIn Initial state
     * \param stateTowardsIn Final state
     * \param exactConnectionOut Set to true if the initial and the final states
     *                           can be connected exactly.
     *
     */
    double evaluateExtensionCost (State_t &stateFromIn, State_t &stateTowardsIn, bool &exactConnectionOut);
    
    /*!
     * \brief Returns a lower bound on the cost to go starting from stateIn
     *
     * A more elaborate description.
     *
     * \param stateIn Starting state
     *
     */
    double evaluateCostToGo (State_t& stateIn);
    
    /*!
     * \brief Returns the trajectory as a list of double arrays, each with dimension getNumDimensions.
     *
     * A more elaborate description.
     *
     * \param stateFromIn Initial state
     * \param stateToIn Final state
     * \param trajectoryOut The list of double arrays that represent the trajectory
     *
     */
    int getTrajectory (State_t& stateFromIn, State_t& stateToIn, std::list<double*>& trajectoryOut);
  };
}

template<class Trajectory_t, class State_t, class region_t>
std::list<region_t*> Holonomic::System<Trajectory_t, State_t, region_t>::obstacles(0,NULL);


#endif
