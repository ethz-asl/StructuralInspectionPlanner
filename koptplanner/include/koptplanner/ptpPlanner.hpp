/*!
 * \file ptpPlanner.hpp
 *
 * More elaborate description
 */
#ifndef _PTPPLANNER_HPP_
#define _PTPPLANNER_HPP_

#include "koptplanner/plan.hpp"
#include "planner_rrts/system_holonomic.h"
#include "koptplanner/TriangleObject.h"
#include "koptplanner/Rotorcraft.h"
#include "koptplanner/FixedWing.h"
#include "GainType.h"
#include <vector>
#include <eigen3/Eigen/Dense>
#include "planner_rrts/rrts.h"

#ifdef __cplusplus
  using namespace RRTstar;
  using namespace Eigen;
  USE_MODEL_NAMESPACE

  typedef MDL_NAMESPACE::region reg_t;
  typedef MDL_NAMESPACE::State state_t;
  typedef MDL_NAMESPACE::Trajectory<state_t> traj_t;
  typedef MDL_NAMESPACE::System<traj_t, state_t, reg_t> sys_t;
  typedef MDL_NAMESPACE::Triangle<sys_t, state_t, StateVector, reg_t> tri_t;
  typedef Planner<state_t, traj_t, sys_t> planner_t;
  typedef Vertex<state_t, traj_t, sys_t> vertex_t;

  /*!
   * \brief RRTsplus class
   *
   * More elaborate description
   */
  class RRTsplus : public Planner<state_t, traj_t, sys_t>
  {
  public:

    /*!
     * \brief RRTsplus constructor
     *
     * More elaborate description
     */
    RRTsplus();

    /*!
     * \brief RRTsplus destructor
     *
     * More elaborate description
     */
    ~RRTsplus();

    /*!
     * \brief Method to extract piece of path to publish
     *
     * More elaborate description
     *
     * \param s Pose at which the path piece ends
     *
     * \return Vector of coordinate vectors that describe the path
     */
    std::vector<std::vector<float> > getPathForPublication(StateVector s);

    /*!
     * \brief Method to extract last visible
     *
     * This method extracts the last pose along the path up the tree
     * from which a certain mesh triangle is visible.
     *
     * \param s Pose at which the path piece ends
     * \param tri Reference to the triangle that has to be visible
     *
     * \return State vector of the last vertex
     */
    StateVector getLastVisible(StateVector s, tri_t * tri);

    /*!
     * \brief Function to evaluate the distance (cost)
     *
     * More elaborate description
     *
     * \param state Goal state
     *
     * \return Distance (cost) to move from root to state
     */
    float evalDist(state_t state);
  };
    

  /*!
   * \brief PTPPlanner class
   *
   * More elaborate description
   */
  class PTPPlanner
  {
  public:

    /*!
     * \brief PTPPlanner constructor
     *
     * More elaborate description
     */
    PTPPlanner();

    /*!
     * \brief PTPPlanner destructor
     *
     * More elaborate description
     */
    ~PTPPlanner();

    /*!
     * \brief Initializing the RRT* planner
     *
     * More elaborate description
     *
     * \param s Root
     * \param centrex Centre of the space x-coordinate aligned
     * \param centrey Centre of the space y-coordinate aligned
     * \param centrez Centre of the space z-coordinate aligned
     * \param sizex Dimension of the space x-coordinate aligned
     * \param sizey Dimension of the space y-coordinate aligned
     * \param sizez Dimension of the space z-coordinate aligned
     */
    void initialize(StateVector,float,float,float,float,float,float);

    /*!
     * \brief Reinitializing the RRT* planner
     *
     * More elaborate description
     *
     * \param s Root
     * \param centrex Centre of the space x-coordinate aligned
     * \param centrey Centre of the space y-coordinate aligned
     * \param centrez Centre of the space z-coordinate aligned
     * \param sizex Dimension of the space x-coordinate aligned
     * \param sizey Dimension of the space y-coordinate aligned
     * \param sizez Dimension of the space z-coordinate aligned
     */
    void reinitialize(StateVector,float,float,float,float,float,float);
    
    /*!
     * \brief Extended RRT* planner
     *
     * More elaborate description
     */
    RRTsplus rrts_;
    
    /*!
     * \brief Reference to a system object
     *
     * More elaborate description
     */
    sys_t* system;
    
    /*!
     * \brief State vector of the root (actual viewpoint)
     *
     * More elaborate description
     */
    StateVector stateVec;
    
    /*!
     * \brief Order of the viewpoints on the tour
     *
     * This static vector contains the ordered viewpoint IDs describing
     * the current best tour
     */
    static std::vector<int> Tour_;
  };

#else
    
  /*!
   * \brief Struct definition for C-code interfacing
   *
   * More elaborate description
   */
  typedef
    struct PTPPlanner
      PTPPlanner;
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__STDC__) || defined(__cplusplus)   /* ANSI C prototypes */

  /*!
   * \brief Evaluate distance
   *
   * This C function interfaces with the cplusplus_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   *
   * \return Distance (cost) to move from first to second viewpoint
   */
  extern int c_callback_function(int,int);

  /*!
   * \brief Evaluate distance
   *
   * This C++ function interfaces with the c_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   *
   * \return Distance (cost) to move from first to second viewpoint
   */
  extern int cplusplus_callback_function(int,int);

  /*!
   * \brief Publish new best tour
   *
   * This C function interfaces with the cplusplus_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   * \param Cost New best cost of tour
   *
   * \return Returns 1 for success
   */
  extern int c_callback_publish(int*,int,GainType);

  /*!
   * \brief Publish new best tour
   *
   * This C++ function interfaces with the c_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   * \param Cost New best cost of tour
   *
   * \return Returns 1 for success
   */
  extern int cplusplus_callback_publish(int*,int,GainType);
#else        /* K&R style */

  /*!
   * \brief Evaluate distance
   *
   * This C function interfaces with the cplusplus_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   *
   * \return Distance (cost) to move from first to second viewpoint
   */
  extern int c_callback_function();

  /*!
   * \brief Evaluate distance
   *
   * This C++ function interfaces with the c_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   *
   * \return Distance (cost) to move from first to second viewpoint
   */
  extern int cplusplus_callback_function();

  /*!
   * \brief Publish new best tour
   *
   * This C function interfaces with the cplusplus_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   * \param Cost New best cost of tour
   *
   * \return Returns 1 for success
   */
  extern int c_callback_publish();

  /*!
   * \brief Publish new best tour
   *
   * This C++ function interfaces with the c_callback_function
   *
   * \param ID ID of first viewpoint
   * \param ID2 ID of second viewpoint
   * \param Cost New best cost of tour
   *
   * \return Returns 1 for success
   */
  extern int cplusplus_callback_publish();
#endif

#ifdef __cplusplus
}
#endif

#endif /* _PTPPLANNER_HPP_ */
