/*!
 * \file ptpPlanner.cpp
 *
 * More elaborate description
 */
#include "ros/ros.h"
#include "koptplanner/plan.hpp"
#include "koptplanner/ptpPlanner.hpp"
#include <assert.h>
#include <cmath>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "koptplanner/inspection.h"
#include "planner_rrts/rrts.h"
#include "planner_rrts/kdtree.h"
#include "planner_rrts/system_holonomic.h"
#include "koptplanner/TriangleObject.hpp"
#include "planner_rrts/rrts.hpp"
#include "planner_rrts/system_holonomic.hpp"
#include "koptplanner/Rotorcraft.hpp"
#include "koptplanner/FixedWing.hpp"
#include "tf/tf.h"
#include <ros/package.h>

#define X_MIN (problemBoundary.center[0]-problemBoundary.size[0]/2)
#define Y_MIN (problemBoundary.center[1]-problemBoundary.size[1]/2)
#define Z_MIN (problemBoundary.center[2]-problemBoundary.size[2]/2)
#define X_MAX (problemBoundary.center[0]+problemBoundary.size[0]/2)
#define Y_MAX (problemBoundary.center[1]+problemBoundary.size[1]/2)
#define Z_MAX (problemBoundary.center[2]+problemBoundary.size[2]/2)

#ifdef __TIMING_INFO__
 extern long time_LKH;
 extern long time_RRTS;
 extern long time_RRTS_req;
#endif
extern long time_start;

extern bool g_closed_tour;

PTPPlanner** plannerArray;
bool plannerArrayBool;

extern ros::Publisher marker_pub;
extern ros::Publisher viewpoint_pub;
extern koptplanner::inspection::Response* res_g;
extern int * reinitRRTs;
extern int maxID;
extern reg_t problemBoundary;
extern StateVector * VP;

extern double g_scale;
extern double g_speed;
extern double g_maxAngularSpeed;
extern double g_cost;
extern string g_tourlength;
extern double g_rrt_scope;
extern int g_rrt_it;
extern int g_rrt_it_init;
extern bool g_lazy_obstacle_check;

std::vector<int> PTPPlanner::Tour_;

using namespace RRTstar;
USE_MODEL_NAMESPACE

RRTsplus::RRTsplus()
{    
  gamma = 1.8;
  
  lowerBoundCost = FLT_MAX;
  lowerBoundVertex = NULL;
  
  kdtree = NULL; 
  
  root = NULL;
  
  numVertices = 0;
  
  system = NULL;
}

RRTsplus::~RRTsplus()
{
  // Delete the kdtree structure
  if (kdtree)
  {
    kd_clear (kdtree);
    kd_free (kdtree);
    kdtree = NULL;
  }
  
  // Delete all the vertices
  for (typename std::list<Vertex <state_t,traj_t,sys_t> * >::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
  {
    delete *iter; *iter = NULL;
  }
}

std::vector<std::vector<float> > RRTsplus::getPathForPublication(StateVector s)
{
  std::vector<std::vector<float> > path;
  state_t endState;
  endState.setNumDimensions(DIMENSIONALITY);
  for(int i = 0; i<DIMENSIONALITY; i++)
    endState[i] = s[i];
  // 2. Compute the set of all near vertices
  std::vector< vertex_t* > vectorNearVertices;
  this->getNearVertices(endState, vectorNearVertices);
  
  // 3. Find the best parent and extend from that parent
  vertex_t* vertexParent = NULL;
  traj_t trajectory;
  bool exactConnection = false;
  
  if (vectorNearVertices.size() == 0)
  {
    // 3.a Extend the nearest
    int run = 0;
    while(true)
    {
      if(run++>100)
      {
        ROS_ERROR("Infeasible connection.");
        std::string pkgPath = ros::package::getPath("koptplanner");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
          ROS_ERROR("Could not open report.log");
        plannerLog << "-->Infeasible connection between viewpoints\n";
        plannerLog << "(" << s[0] << ", " << s[1] << ", " << s[2] << ") and ";
        plannerLog << "(" << this->getRootVertex().getState()[0] << ", " << this->getRootVertex().getState()[1] << ", " << this->getRootVertex().getState()[2] << ")\n";
        plannerLog.close();
        koptError = CONNECTION_INFEASIBILITY;
        break;
      }
      if (this->getNearestVertex(endState, vertexParent) <= 0) 
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
          return path;
        continue;
        ROS_ERROR("no near vertex");
      }
      if (this->system->extendTo(vertexParent->getState(), endState, trajectory, exactConnection) <= 0)
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
          return path;
        continue;
        ROS_ERROR("not extendable");
      }
      break;
    }
  }
  else
  {
    // 3.b Extend the best parent within the near vertices
    int run = 0;
    while(true)
    {
      if(run++>100)
      {
        ROS_ERROR("Infeasible connection.");
        std::string pkgPath = ros::package::getPath("koptplanner");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
          ROS_ERROR("Could not open report.log");
        plannerLog << "-->Infeasible connection between viewpoints\n";
        plannerLog << "(" << s[0] << ", " << s[1] << ", " << s[2] << ") and ";
        plannerLog << "(" << this->getRootVertex().getState()[0] << ", " << this->getRootVertex().getState()[1] << ", " << this->getRootVertex().getState()[2] << ")\n";
        plannerLog.close();
        koptError = CONNECTION_INFEASIBILITY;
        break;
      }
      if (this->findBestParent (endState, vectorNearVertices, vertexParent, trajectory, exactConnection) <= 0)
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
          return path;
        continue;
        ROS_ERROR("no best parent");
      }
      break;
    }
  }
  std::list<double*> trajectory1;
  this->system->getTrajectory (vertexParent->getState(), endState, trajectory1);
  
  trajectory1.reverse ();
  for (std::list<double*>::iterator iter = trajectory1.begin(); iter != trajectory1.end(); iter++)
  {
    double *stateArrFromParentCurr = *iter;

    std::vector<float> stateArrCurr2;
    for(int i=0; i<numDimensions; ++i)
    {
      stateArrCurr2.push_back(stateArrFromParentCurr[i]);
    }
    path.push_back(stateArrCurr2);
    
    delete [] stateArrFromParentCurr;
  }

  vertex_t* vertexCurr = vertexParent;
  
  
  while (vertexCurr)
  {
    state_t& stateCurr = vertexCurr->getState();
    
    std::vector<float> stateArrCurr;
    for(int i=0; i<this->numDimensions; ++i)
    { 
        stateArrCurr.push_back(stateCurr[i]);
    }
    
    path.push_back(stateArrCurr);
    
    vertexParent = &(vertexCurr->getParent()); 
    
    if (vertexParent != NULL)
    {
      state_t& stateParent = vertexParent->getState();
      
      std::list<double*> trajectory2;
      this->system->getTrajectory (stateParent, stateCurr, trajectory2);
      
      trajectory2.reverse ();
      for (std::list<double*>::iterator iter = trajectory2.begin(); iter != trajectory2.end(); iter++)
      {
        if(iter != trajectory2.begin())
        {
          double *stateArrFromParentCurr = *iter;
          std::vector<float> stateArrCurr2;
          for(int i=0; i<numDimensions; ++i)
          {
            stateArrCurr2.push_back(stateArrFromParentCurr[i]);
          }
          path.push_back(stateArrCurr2);
          
          delete [] stateArrFromParentCurr;
        }
      }
    }

    vertexCurr = vertexParent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

float RRTsplus::evalDist(state_t state)
{
#ifdef __TIMING_INFO__
  timeval time;
  gettimeofday(&time, NULL);
  time_RRTS_req -= time.tv_sec * 1000000 + time.tv_usec;
#endif
  // 2. Compute the set of all near vertices
  std::vector< vertex_t* > vectorNearVertices;
  this->getNearVertices(state, vectorNearVertices);
  
  // 3. Find the best parent and extend from that parent
  vertex_t* vertexParent = NULL;
  traj_t trajectory;
  bool exactConnection = false;
  
  if (vectorNearVertices.size() == 0)
  {
    // 3.a Extend the nearest
    int run = 0;
    while(true)
    {
      if(run++>100)
      {
        ROS_ERROR("Infeasible connection.");
        std::string pkgPath = ros::package::getPath("koptplanner");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
          ROS_ERROR("Could not open report.log");
        plannerLog << "-->Infeasible connection between viewpoints\n";
        plannerLog << "(" << state[0] << ", " << state[1] << ", " << state[2] << ") and ";
        plannerLog << "(" << this->getRootVertex().getState()[0] << ", " << this->getRootVertex().getState()[1] << ", " << this->getRootVertex().getState()[2] << ")\n";
        plannerLog.close();
        koptError = CONNECTION_INFEASIBILITY;
        break;
      }
      if (this->getNearestVertex(state, vertexParent) <= 0) 
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
        {
          #ifdef __TIMING_INFO__
          gettimeofday(&time, NULL);
          time_RRTS_req += time.tv_sec * 1000000 + time.tv_usec;
          #endif
          return 1e7;
        }
        continue;
      }
      if (this->system->extendTo(vertexParent->getState(), state, trajectory, exactConnection) <= 0)
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
        {
          #ifdef __TIMING_INFO__
          gettimeofday(&time, NULL);
          time_RRTS_req += time.tv_sec * 1000000 + time.tv_usec;
          #endif
          return 1e7;
        }
        continue;
      }
      break;
    }
  }
  else
  {
    // 3.b Extend the best parent within the near vertices
    int run = 0;
    while(true)
    {
      if(run++>100)
      {
        ROS_ERROR("Infeasible connection.");
        std::string pkgPath = ros::package::getPath("koptplanner");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
          ROS_ERROR("Could not open report.log");
        plannerLog << "-->Infeasible connection between viewpoints\n";
        plannerLog << "(" << state[0] << ", " << state[1] << ", " << state[2] << ") and ";
        plannerLog << "(" << this->getRootVertex().getState()[0] << ", " << this->getRootVertex().getState()[1] << ", " << this->getRootVertex().getState()[2] << ")\n";
        plannerLog.close();
        koptError = CONNECTION_INFEASIBILITY;
        break;
      }
      this->getNearVertices(state, vectorNearVertices);
      if (this->findBestParent (state, vectorNearVertices, vertexParent, trajectory, exactConnection) <= 0)
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
        {
          #ifdef __TIMING_INFO__
          gettimeofday(&time, NULL);
          time_RRTS_req += time.tv_sec * 1000000 + time.tv_usec;
          #endif
          return 1e7;
        }
        continue;
      }
      break;
    }
  }
  float ret = trajectory.evaluateCost() + vertexParent->getCost();
#ifdef __TIMING_INFO__
  gettimeofday(&time, NULL);
  time_RRTS_req += time.tv_sec * 1000000 + time.tv_usec;
#endif
#ifndef USE_FIXEDWING_MODEL
  ret = std::max(ret / g_speed, ANGABS(state[3] - this->root->getState()[3])/g_maxAngularSpeed);
#endif
  return ret;
}

StateVector RRTsplus::getLastVisible(StateVector s, tri_t * tri)
{
  if(tri->Fixpoint)
    return s;
  state_t state;
  state.setNumDimensions(DIMENSIONALITY);
  for(int i = 0; i<DIMENSIONALITY; i++)
    state[i] = s[i];
  // 2. Compute the set of all near vertices
  std::vector< vertex_t* > vectorNearVertices;
  this->getNearVertices(state, vectorNearVertices);
  
  // 3. Find the best parent and extend from that parent
  vertex_t* vertexParent = NULL;
  traj_t trajectory;
  bool exactConnection = false;
  
  if (vectorNearVertices.size() == 0)
  {
    // 3.a Extend the nearest
    int run = 0;
    while(true)
    {
      if(run++>100)
      {
        ROS_ERROR("Infeasible connection.");
        std::string pkgPath = ros::package::getPath("koptplanner");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
          ROS_ERROR("Could not open report.log");
        plannerLog << "-->Infeasible connection between viewpoints\n";
        plannerLog << "(" << s[0] << ", " << s[1] << ", " << s[2] << ") and ";
        plannerLog << "(" << this->getRootVertex().getState()[0] << ", " << this->getRootVertex().getState()[1] << ", " << this->getRootVertex().getState()[2] << ")\n";
        plannerLog.close();
        koptError = CONNECTION_INFEASIBILITY;
        break;
      }
      if (this->getNearestVertex(state, vertexParent) <= 0) 
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
          return *(new StateVector);
        continue;
      }
      if (this->system->extendTo(vertexParent->getState(), state, trajectory, exactConnection) <= 0)
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
          return *(new StateVector);
        continue;
      }
      break;
    }
  }
  else
  {
    // 3.b Extend the best parent within the near vertices
    int run = 0;
    while(true)
    {
      if(run++>100)
      {
        ROS_ERROR("Infeasible connection.");
        std::string pkgPath = ros::package::getPath("koptplanner");
        std::fstream plannerLog;
        plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
        if(!plannerLog.is_open())
          ROS_ERROR("Could not open report.log");
        plannerLog << "-->Infeasible connection between viewpoints\n";
        plannerLog << "(" << s[0] << ", " << s[1] << ", " << s[2] << ") and ";
        plannerLog << "(" << this->getRootVertex().getState()[0] << ", " << this->getRootVertex().getState()[1] << ", " << this->getRootVertex().getState()[2] << ")\n";
        plannerLog.close();
        koptError = CONNECTION_INFEASIBILITY;
        break;
      }
      this->getNearVertices(state, vectorNearVertices);
      if (this->findBestParent (state, vectorNearVertices, vertexParent, trajectory, exactConnection) <= 0)
      {
        for(int i = 0; i<g_rrt_it; i++)
          this->iteration();
        if(this->numVertices>5*g_rrt_it+1)
          return *(new StateVector);
        continue;
      }
      break;
    }
  }
  vertex_t * vert;
  while(vertexParent != NULL)
  {
    vert = vertexParent;
    state_t sOut = vertexParent->getState();
    StateVector sVec;
    for(int i = 0; i<DIMENSIONALITY; i++)
      sVec[i] = sOut[i];
    if(!tri->isVisible(sVec))
    {
      return sVec;
    }
    vertexParent = &(vertexParent->getParent());
  }
  state_t sOut = vert->getState();
  StateVector sVec; 
  for(int i = 0; i<DIMENSIONALITY; i++)
    sVec[i] = sOut[i];
  return sVec;
}

PTPPlanner::PTPPlanner()
{
  system = NULL;
}

PTPPlanner::~PTPPlanner()
{
  delete system;
  system = NULL;
}

void PTPPlanner::initialize(StateVector s, float centrex, float centrey, float centrez, float sizex, float sizey, float sizez)
{
  stateVec = s;
#ifdef __TIMING_INFO__
  timeval time;
  gettimeofday(&time, NULL);
  time_RRTS -= time.tv_sec * 1000000 + time.tv_usec;
#endif

  // DIMENSIONALITY dimensional configuration space
  if(system)
    delete system;
  system = new sys_t;
  system->setNumDimensions(DIMENSIONALITY);

  // Define the operating region
  system->regionOperating.setNumDimensions(DIMENSIONALITY);
  system->regionOperating.center[0] = (std::min((float)X_MAX,centrex+sizex/2)+std::max((float)X_MIN,centrex-sizex/2))/2;
  system->regionOperating.center[1] = (std::min((float)Y_MAX,centrey+sizey/2)+std::max((float)Y_MIN,centrey-sizey/2))/2;
  system->regionOperating.center[2] = (std::min((float)Z_MAX,centrez+sizez/2)+std::max((float)Z_MIN,centrez-sizez/2))/2;
  system->regionOperating.size[0] = std::min(2*std::min(fabs(system->regionOperating.center[0]-X_MAX),fabs(system->regionOperating.center[0]-X_MIN)),(double)sizex);
  system->regionOperating.size[1] = std::min(2*std::min(fabs(system->regionOperating.center[1]-Y_MAX),fabs(system->regionOperating.center[1]-Y_MIN)),(double)sizey);
  system->regionOperating.size[2] = std::min(2*std::min(fabs(system->regionOperating.center[2]-Z_MAX),fabs(system->regionOperating.center[2]-Z_MIN)),(double)sizez);
#if DIMENSIONALITY==4
  system->regionOperating.center[3] = 0.0;
  system->regionOperating.size[3] = 2.0*M_PI;
#endif
#if DIMENSIONALITY>4
  system->regionOperating.center[3] = 0.0;
  system->regionOperating.size[3] = 0.00001;
  system->regionOperating.center[4] = 0.0;
  system->regionOperating.size[4] = 2.0*g_maxClimbSinkRate;
  system->regionOperating.center[5] = 0.0;
  system->regionOperating.size[5] = 2.0*M_PI;
#endif
  // Define the goal region (not to be reached)
  system->regionGoal.setNumDimensions(DIMENSIONALITY);
  for(int i = 0; i < DIMENSIONALITY; i++)
  {
    system->regionGoal.center[i] = 0.0;
    system->regionGoal.size[i] = 0.0;
  }

  // Add the system to the planner
  rrts_.setSystem(*system);

  // Set up the root vertex
  vertex_t &root = rrts_.getRootVertex();
  state_t &rootState = root.getState();
  for(int i = 0; i<DIMENSIONALITY; i++)
    rootState[i] = stateVec[i];

  // Initialize the planner
  rrts_.initialize();

  // This parameter should be larger than 1.5 for asymptotic
  //   optimality. Larger values will weigh on optimization
  //   rather than exploration in the RRT* algorithm. Lower
  //   values, such as 0.1, should recover the RRT.
  rrts_.setGamma(1.8);

  // Run the algorithm for g_rrt_it_init iteartions
  for (int i = 0; i < g_rrt_it_init; i++)
  {
    rrts_.iteration();
  }
#ifdef __TIMING_INFO__
  gettimeofday(&time, NULL);
  time_RRTS += time.tv_sec * 1000000 + time.tv_usec;
#endif
}

void PTPPlanner::reinitialize(StateVector s, float centrex, float centrey, float centrez, float sizex, float sizey, float sizez)
{
  initialize(s, centrex, centrey, centrez, sizex, sizey, sizez);
}

int cplusplus_callback_publish(int* Tour, int Dim, GainType Cost)
{
  std::vector<int> T;
  int q = 0;
  while(q<Dim && Tour[q] != 1)
    q++;
  for(int l = 0; l<q; l++)
  {
    if(l+q<Dim)
    {
      if(Tour[l+q] != 0)
        T.push_back(Tour[l+q]);
    }
    else
    {
      if(Tour[l+q-Dim] != 0)
        T.push_back(Tour[l+q-Dim]);
    }
  }
  for(int l = q; l<=Dim; l++)
  {
    if(l+q<Dim)
    {
      if(Tour[l+q] != 0)
        T.push_back(Tour[l+q]);
    }
    else
    {
      if(Tour[l+q-Dim] != 0)
        T.push_back(Tour[l+q-Dim]);
    }
  }
  std::vector<int>::iterator itT;
  for(itT = T.begin(); itT != T.end()-1 && *itT != *(itT+1); itT++) {}
  T.erase(itT);
    
#ifdef __ATSP__
  Dim /=2;
  T.erase(T.end()-1);
#endif

  if(Tour)
    PTPPlanner::Tour_ = T;
  
#ifndef USE_FIXEDWING_MODEL
  double cost_new = ((double)(int)Cost)/(g_scale);
#else
  double cost_new = ((double)(int)Cost)/(g_speed*g_scale);
#endif
  if(g_cost > cost_new)
  {
    ROS_INFO("New tour cost = %2.2f", cost_new);
    g_cost = cost_new;;
  }
  else
  {
    return 1;
  }
  res_g->inspectionPath.poses.clear();
  std::fstream file;
  file.open(g_tourlength.c_str(), std::ios::app | std::ios::out);
  if(file.is_open())
  {
    timeval time;
    gettimeofday(&time, NULL);
    long millisec = time.tv_sec * 1000 + time.tv_usec / 1000;
    file << ((double)(int)Cost)/((double)g_scale) << ", " << millisec - time_start << ";\n";
    file.close();
  }
  else
  {
    ROS_WARN("tourlength file not found");
  }
  int k = 0;
  bool directionReverse = T[1] == Dim;
  for(int i = ((T[1] == Dim && !g_closed_tour) ? 1 : 0); i<T.size() - ((T[T.size()-1] == Dim && !g_closed_tour) ? 1 : 0); i++)
  {
    std::vector<std::vector<float> > path;
    if(!(i<T.size()-1))
    {
#ifndef USE_FIXEDWING_MODEL
      if(T[i] == 1)
#else
      if(false)
#endif
      {
        path = plannerArray[T[0]-1]->rrts_.getPathForPublication(plannerArray[i]->stateVec);
        std::vector<float> tmp;
        for(int l = 0; l<DIMENSIONALITY; l++)
          tmp.push_back(plannerArray[i]->stateVec[l]);
        path.push_back(tmp);
        std::reverse(path.begin(), path.end());
      }
      else
      {
        path = plannerArray[T[i]-1]->rrts_.getPathForPublication(plannerArray[T[0]-1]->stateVec);
        std::vector<float> tmp;
        for(int l = 0; l<DIMENSIONALITY; l++)
          tmp.push_back(plannerArray[T[0]-1]->stateVec[l]);
        path.push_back(tmp);
      }
    }
    else
    {
#ifndef USE_FIXEDWING_MODEL
      if(T[i] == 1)
#else
      if(false)
#endif
      {
        path = plannerArray[T[i+1]-1]->rrts_.getPathForPublication(plannerArray[0]->stateVec);
        std::vector<float> tmp;
        for(int l = 0; l<DIMENSIONALITY; l++)
          tmp.push_back(plannerArray[0]->stateVec[l]);
        path.push_back(tmp);
        std::reverse(path.begin(), path.end());
      }
      else
      {
        path = plannerArray[T[i]-1]->rrts_.getPathForPublication(plannerArray[T[i+1]-1]->stateVec);
        std::vector<float> tmp;
        for(int l = 0; l<DIMENSIONALITY; l++)
          tmp.push_back(plannerArray[T[i+1]-1]->stateVec[l]);
        path.push_back(tmp);
      }
    }

    for(std::vector<std::vector<float> >::iterator it = path.begin(); it!=path.end(); it++)
    {
      geometry_msgs::PoseStamped tempPose;
      tempPose.pose.position.x = (*it)[0];
      tempPose.pose.position.y = (*it)[1];
      tempPose.pose.position.z = (*it)[2];
#if DIMENSIONALITY>4
      tf::Quaternion q = tf::createQuaternionFromRPY((*it)[3],(*it)[4],(*it)[5]);
#else
      tf::Quaternion q = tf::createQuaternionFromRPY(0,0,(*it)[3]);
#endif
      tempPose.pose.orientation.x = q.x();
      tempPose.pose.orientation.y = q.y();
      tempPose.pose.orientation.z = q.z();
      tempPose.pose.orientation.w = q.w();
      tempPose.header.stamp = ros::Time::now();
      tempPose.header.seq = k;
      tempPose.header.frame_id = "/kopt_frame";
      res_g->inspectionPath.poses.push_back(tempPose);

      k++;
    }
  }
  if(directionReverse)
  {
    std::reverse(res_g->inspectionPath.poses.begin(),res_g->inspectionPath.poses.end());
  }
  res_g->inspectionPath.header.stamp = ros::Time::now();
  res_g->inspectionPath.header.seq = 1;
  res_g->inspectionPath.header.frame_id = "/kopt_frame";
  marker_pub.publish(res_g->inspectionPath);
  std::fstream f;
  std::string pathAssemb = ros::package::getPath("koptplanner");
  pathAssemb += "/data/latestPath.csv";
  f.open(pathAssemb.c_str(), std::ios::out);
  if(!f.is_open())
    ROS_ERROR("path not found");
  f << "";
  for(std::vector<geometry_msgs::PoseStamped>::iterator it = res_g->inspectionPath.poses.begin(); it != res_g->inspectionPath.poses.end(); it++)
  {
    tf::Pose pose;
    tf::poseMsgToTF(it->pose, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());
    f << it->pose.position.x << "," << it->pose.position.y << "," << it->pose.position.z << ",0,0," << yaw_angle << "\n";
  }
  f.close();
  return 1;
}

int cplusplus_callback_function(int ID, int ID2)
{
  ID%=maxID;
  ID2%=maxID;
#ifdef __TIMING_INFO__
  timeval time;
  gettimeofday(&time, NULL);
  time_LKH += time.tv_sec * 1000000 + time.tv_usec;
#endif
  long ret = INT_MAX;

  if(!plannerArrayBool) // allocate
  {
    plannerArray = new PTPPlanner*[maxID];
    for(int j = 0; j<maxID; j++)
    {
      plannerArray[j] = NULL;
    }
  }
  if(reinitRRTs[ID] == 1&&plannerArray[ID]) // delete for reinit
  {
    delete plannerArray[ID];
    plannerArray[ID] = NULL;
  }
  if(reinitRRTs[ID] == 2&&plannerArray[ID])
  {
    for(int j = 0; j<g_rrt_it_init; j++)
      plannerArray[ID]->rrts_.iteration();
    reinitRRTs[ID] = 0;
  }
  if(!plannerArrayBool) // build first tree, that is not built otherwise
  {
    plannerArrayBool = true;
    StateVector tmp = VP[0];
    plannerArray[0] = new PTPPlanner();
    plannerArray[0]->initialize(tmp,tmp[0],tmp[1],tmp[2],2.0*g_rrt_scope,2.0*g_rrt_scope,2.0*g_rrt_scope);
  }
  if(!plannerArray[ID]) // init
  {
    plannerArray[ID] = new PTPPlanner();
    StateVector tmp = VP[ID];
    plannerArray[ID]->stateVec = tmp;
    plannerArray[ID]->initialize(tmp,tmp[0],tmp[1],tmp[2],2.0*g_rrt_scope,2.0*g_rrt_scope,2.0*g_rrt_scope);
    reinitRRTs[ID] = 0;
  }
  double distLazy = sqrt( pow(VP[ID][0] - VP[ID2][0],2.0) + pow(VP[ID][1] - VP[ID2][1],2.0) + pow(VP[ID][2] - VP[ID2][2],2.0) );
  if(distLazy>g_rrt_scope)
  {
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_LKH -= time.tv_sec * 1000000 + time.tv_usec;
#endif
    bool bCollision = false;
    /* collision check also for lazy connections */
    if(g_lazy_obstacle_check)
    {
      for(double it = 0; it < 1; it += g_discretization_step/distLazy)
      {
        double * tmp = new double[DIMENSIONALITY];
        for(int i = 0; i<DIMENSIONALITY; i++)
          tmp[i] = VP[ID][i]*it+VP[ID2][i]*(1-it);
        if(plannerArray[ID]->rrts_.system->IsInCollision(tmp))
        {
          bCollision = true;
          break;
        }
        delete[] tmp;
      }
    }
    if(!bCollision)
    {
#ifndef USE_FIXEDWING_MODEL
      return (int) std::min((double)INT_MAX-1,std::max(distLazy/g_speed, ANGABS(VP[ID][3]-VP[ID2][3])/g_maxAngularSpeed) * g_scale);
#else
      return (int) std::min((double)INT_MAX-1,(((double)(distLazy * g_scale)) + 0.5));
#endif
    }
  }
  state_t state;
  state.setNumDimensions(DIMENSIONALITY);
  for(int j = 0; j<DIMENSIONALITY; j++)
    state[j] = VP[ID2][j];
  ret = (int)std::min((double)INT_MAX-1,(plannerArray[ID]->rrts_.evalDist(state)*g_scale+0.5)); // distance
#ifdef __TIMING_INFO__
  gettimeofday(&time, NULL);
  time_LKH -= time.tv_sec * 1000000 + time.tv_usec;
#endif
  return ret;
}
