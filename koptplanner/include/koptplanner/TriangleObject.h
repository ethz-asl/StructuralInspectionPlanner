/*!
 * \file TriangleObject.h
 *
 * More elaborate description
 */
#ifndef __TRIANGLE_OBJECT_H__
#define __TRIANGLE_OBJECT_H__

#include "koptplanner/plan.hpp"
#include <eigen3/Eigen/Dense>
#include "optec/qpOASES.hpp"
#include "float.h"

using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace Holonomic
{
    
  /*!
   * \brief Triangle class
   *
   * More elaborate description
   */
  template<class System_t, class State_t, class Vector_t, class region_t>
  class Triangle
  {
  protected:
    
    /*!
     * \brief Static parameter maximal incidence angle (measured to plain)
     *
     * More elaborate description
     */
    static double incidenceAngle;
    
    /*!
     * \brief Static parameter minimally allowed distance for inspection
     *
     * More elaborate description
     */
    static double minDist;
    
    /*!
     * \brief Static parameter maximally allowed distance for inspection
     *
     * More elaborate description
     */
    static double maxDist;
  public:
    
    /*!
     * \brief Triangle constructor
     *
     * More elaborate description
     */
    Triangle();
    
    /*!
     * \brief Triangle destructor
     *
     * More elaborate description
     */
    ~Triangle();
        
    /*!
     * \brief Static function to set inspection parameters
     *
     * More elaborate description
     * 
     * \param incidenceAngleIn Maximal incidence angle
     * \param minDistIn Minimal inspection distance
     * \param maxDistIn Maximal inspection distance
     */
    static void setParam(double incidenceAngleIn, double minDistIn, double maxDistIn);
        
    /*!
     * \brief Check whether object is visible
     *
     * More elaborate description
     * 
     * \param state State to be checked
     * 
     * \return If visible
     */
    bool isVisible(State_t state);
        
    /*!
     * \brief Check whether object is visible
     *
     * More elaborate description
     * 
     * \param s State to be checked
     * 
     * \return If visible
     */
    bool isVisible(Vector_t s);
        
    /*!
     * \brief Purely virtual viewpoint sampling function
     *
     * More elaborate description
     * 
     * \param state1 Preceding state
     * \param state2 Succeeding state
     * \param statePrev Previous state
     * 
     * \return Newly sampled viewpoint state
     */
    virtual Vector_t dualBarrierSampling(Vector_t* state1, Vector_t* state2, Vector_t* statePrev) = 0;
        
    /*!
     * \brief Purely virtual function that initializes the viewpoint sampler
     *
     * More elaborate description
     */
    virtual void init() = 0;
        
    /*!
     * \brief Collision checker
     *
     * Oposed to the collision checking method in the vehicle system class, this
     * function also performs 'ray shooting', checking whether the triangle can
     * actually be inspected.
     * 
     * \param State to be checked
     * 
     * \return stateIn Reference to obstacle that is hit (NULL otherwise)
     */
    region_t* IsInCollision (Vector_t &stateIn);
        
    /*!
     * \brief Describes whether all triangle objects have been initialized
     *
     * More elaborate description
     */
    static bool initialized;
        
    /*!
     * \brief Is object a surface or a fixpoint?
     *
     * More elaborate description
     */
    bool Fixpoint;
        
    /*!
     * \brief Corner 1 of the triangle
     *
     * More elaborate description
     */
    Vector3f x1;
        
    /*!
     * \brief Corner 2 of the triangle
     *
     * More elaborate description
     */
    Vector3f x2;
        
    /*!
     * \brief Corner 3 of the triangle
     *
     * More elaborate description
     */
    Vector3f x3;
        
    /*!
     * \brief Normal of the triangle
     *
     * 2-norm of the vector corresponds to the area of the triangle surface
     */
    Vector3f a;
        
    /*!
     * \brief Normal of the triangle
     *
     * 2-norm of the vector is 1
     */
    Vector3f aabs;
        
    /*!
     * \brief Normal of the first separating hyperplane defined by the incidence angle
     *
     * More elaborate description
     */
    Vector3f n1;
        
    /*!
     * \brief Normal of the second separating hyperplane defined by the incidence angle
     *
     * More elaborate description
     */
    Vector3f n2;
        
    /*!
     * \brief Normal of the third separating hyperplane defined by the incidence angle
     *
     * More elaborate description
     */
    Vector3f n3;
        
    /*!
     * \brief H-matrix weighting the squared terms of the optimization problem
     *
     * More elaborate description
     */
    real_t* H;
        
    /*!
     * \brief A-matrix weighting the inequality constraints of the optimization problem
     *
     * More elaborate description
     */
    real_t* A;
        
    /*!
     * \brief d-vector weighting the linear terms of the optimization problem
     *
     * More elaborate description
     */
    real_t* d;
        
    /*!
     * \brief lbA-matrix weighting the lower bounds of the optimization problem
     *
     * More elaborate description
     */
    real_t* lbA;
        
    /*!
     * \brief ubA-matrix weighting the upper bounds of the optimization problem
     *
     * More elaborate description
     */
    real_t* ubA;
        
    /*!
     * \brief QP solver
     *
     * More elaborate description
     */
    QProblem * VPSolver;
  };

}

template<class System_t, class State_t, class Vector_t, class region_t>
double Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::incidenceAngle = M_PI/3;
template<class System_t, class State_t, class Vector_t, class region_t>
double Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::minDist = 0;
template<class System_t, class State_t, class Vector_t, class region_t>
double Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::maxDist = DBL_MAX;
template<class System_t, class State_t, class Vector_t, class region_t>
bool Holonomic::Triangle<System_t, State_t, Vector_t, region_t>::initialized = false;

#endif // __TRIANGLE_OBJECT_H__
