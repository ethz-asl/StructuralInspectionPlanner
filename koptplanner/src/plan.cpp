/*!
 * \file plan.cpp
 *
 * More elaborate description
 */
#include "ros/ros.h"
#include <sys/time.h>
#include <stdlib.h>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Path.h"
#include <std_msgs/Int32.h>
#include "koptplanner/plan.hpp"
#include "planner_rrts/system_holonomic.h"
#include "koptplanner/TriangleObject.h"
#include "koptplanner/Rotorcraft.h"
#include "koptplanner/FixedWing.h"
#include "koptplanner/TriangleObject.hpp"
#include "planner_rrts/rrts.hpp"
#include "planner_rrts/system_holonomic.hpp"
#include "koptplanner/Rotorcraft.hpp"
#include "koptplanner/FixedWing.hpp"
#include "koptplanner/ptpPlanner.hpp"
#include "LKH.h"
#include "koptplanner/inspection.h"
#include "planner_rrts/system_holonomic.h"
#include "optec/qpOASES.hpp"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <shape_msgs/Plane.h>
#include <ros/package.h>

#ifdef __TIMING_INFO__
 long time_DBS;
 long time_RRTS;
 long time_RRTS_req;
 long time_LKH;
#endif
long time_start;

USE_MODEL_NAMESPACE

// Static variables in the functions have to be reset, use bools (int) to trigger:
int Gain23Static;
int GreedyTourmark;
int SFCTourRank;
int ReadPenaltiesStatic;

extern bool plannerArrayBool;
extern PTPPlanner** plannerArray;
int * reinitRRTs;
int maxID;
reg_t problemBoundary;
StateVector * VP;

ros::Publisher marker_pub;
ros::Publisher viewpoint_pub;
koptplanner::inspection::Response* res_g;
double ** lookupTable;
double g_speed;
double g_maxAngularSpeed;
double g_camAngleHorizontal;
double g_camAngleVertical;
double g_camPitch;
double g_maxClimbSinkRate;

double g_scale;
double g_rrt_scope;
int g_rrt_it;
int g_rrt_it_init;
int g_max_obstacle_depth;
double g_discretization_step;
double g_angular_discretization_step;
double g_const_A;
double g_const_B;
double g_const_C;
double g_const_D;
bool g_lazy_obstacle_check;
double g_security_distance;

bool g_closed_tour;
double g_cost;
string g_tourlength;
koptError_t koptError;
double g_max_obs_dim;

bool plan(koptplanner::inspection::Request  &req,
    koptplanner::inspection::Response &res)
{
  koptError = SUCCESSFUL;
  /* loading the parameters */
#ifdef USE_FIXEDWING_MODEL
  if(!ros::param::get("~/fixedwing/speed", g_speed))
    koptError = MISSING_PARAMETER;
  if(!ros::param::get("~/fixedwing/Rmin", sys_t::r_min))
    koptError = MISSING_PARAMETER;
  if(!ros::param::get("~/fixedwing/maxClimbSinkRate", g_maxClimbSinkRate))
    koptError = MISSING_PARAMETER;
#else
  if(!ros::param::get("~/rotorcraft/maxSpeed", g_speed))
    koptError = MISSING_PARAMETER;
  if(!ros::param::get("~/rotorcraft/maxAngularSpeed", g_maxAngularSpeed))
    koptError = MISSING_PARAMETER;
#endif
  if(!ros::param::get("~/camera/horizontal", g_camAngleHorizontal))
      koptError = MISSING_PARAMETER;
  if(!ros::param::get("~/camera/vertical", g_camAngleVertical))
      koptError = MISSING_PARAMETER;
  if(!ros::param::get("~/camera/pitch", g_camPitch))
      koptError = MISSING_PARAMETER;
  if(koptError == MISSING_PARAMETER)
  {
    ROS_ERROR("Missing parameter.");
    std::string pkgPath = ros::package::getPath("koptplanner");
    std::fstream plannerLog;
    plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
    if(!plannerLog.is_open())
      ROS_ERROR("Could not open report.log");
    plannerLog << "-->Missing parameter. Error ocurred while loading parameters from parameter file 'koptParam.yaml'.\n";
    plannerLog.close();
    return true;
  }
  /* loading the optional parameters */
#ifdef USE_FIXEDWING_MODEL
  double vp_tol = 2.0e-6*sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1])+SQ(req.spaceSize[2]));
  g_rrt_scope = 0.25*sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1])+SQ(req.spaceSize[2]));
  g_rrt_it = 100;
  g_rrt_it_init = 0;
  g_max_obstacle_depth = 3;
  g_discretization_step = 1.0e-2*sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1]));
  g_angular_discretization_step = 0.2;
  g_const_A = 1.0e6;
  g_const_B = 3.0;
  g_const_C = 1.0e12;
  g_const_D = 1.0;
  g_lazy_obstacle_check = false;
  g_security_distance = sys_t::r_min;
#elif defined USE_ROTORCRAFT_MODEL
  double vp_tol = 2.0e-6*sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1])+SQ(req.spaceSize[2]));
  g_rrt_scope = 0.125*sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1])+SQ(req.spaceSize[2]));
  g_rrt_it = 50;
  g_rrt_it_init = 0;
  g_max_obstacle_depth = 3;
  g_discretization_step = 5.0e-3*sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1]));
  g_angular_discretization_step = 0.2;
  g_const_A = 1.0; // NA
  g_const_B = 1.0; // NA
  g_const_C = 1.0; // NA
  g_const_D = 1.0;
  g_lazy_obstacle_check = false;
  g_security_distance = 2.0;
#else
  a_model_has_to_be_defined;
#endif
  ros::param::get("~/algorithm/vp_tol", vp_tol);
  ros::param::get("~/algorithm/rrt_scope", g_rrt_scope);
  ros::param::get("~/algorithm/rrt_it", g_rrt_it);
  ros::param::get("~/algorithm/rrt_it_init", g_rrt_it_init);
  ros::param::get("~/algorithm/max_obstacle_depth", g_max_obstacle_depth);
  ros::param::get("~/algorithm/discretization_step", g_discretization_step);
  ros::param::get("~/algorithm/angular_discretization_step", g_angular_discretization_step);
  ros::param::get("~/algorithm/const_A", g_const_A);
  g_const_A /= g_maxClimbSinkRate;
  ros::param::get("~/algorithm/const_B", g_const_B);
  ros::param::get("~/algorithm/const_C", g_const_C);
  ros::param::get("~/algorithm/const_D", g_const_D);
  ros::param::get("~/algorithm/lazy_obstacle_check", g_lazy_obstacle_check);
  ros::param::get("~/algorithm/security_distance", g_security_distance);
  
#ifdef USE_FIXEDWING_MODEL
  g_scale = 1.0e5/sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1])+SQ(req.spaceSize[2]));
#else
  g_scale = g_speed*1.0e5/sqrt(SQ(req.spaceSize[0])+SQ(req.spaceSize[1])+SQ(req.spaceSize[2]));
#endif
  
  if(lookupTable)
  {
    for(int i = 0; i < LOOKUPTABLE_SIZE; i++)
      delete[] lookupTable[i];
    delete[] lookupTable;
    lookupTable = NULL;
  }

  std::string pkgPath = ros::package::getPath("koptplanner");
  g_tourlength = pkgPath + "/data/tourlength.m";

  /* preparing log file */
  std::fstream plannerLog;
  plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::out);
  plannerLog << "Inspection path planner version 1.0.0\nScenario:\n";
  plannerLog << "Number of facets:\t" << req.inspectionArea.size() << "\n";
  plannerLog << "Number of obstacles:\t" << req.obstacles.size() << "\n";
  plannerLog << "Space center:\t\t(" << req.spaceCenter[0] << ", " << req.spaceCenter[1] << ", " << req.spaceCenter[2] << ")\n";
  plannerLog << "Space size:\t\t(" << req.spaceSize[0] << ", " << req.spaceSize[1] << ", " << req.spaceSize[2] << ")\n";
  if(req.requiredPoses.size()>0)
    plannerLog << "Starting point:\t\t(" << req.requiredPoses[0].position.x << ", " << req.requiredPoses[0].position.y << ", " << req.requiredPoses[0].position.z << ")\n";
  if(req.requiredPoses.size()>1)
    plannerLog << "Final point:\t\t(" << req.requiredPoses[1].position.x << ", " << req.requiredPoses[1].position.y << ", " << req.requiredPoses[1].position.z << ")\n";
  plannerLog << "Angle of incidence:\t" << req.incidenceAngle <<"rad\n";
  plannerLog << "Range:\t\t\t(" << req.minDist << ", " << req.maxDist << ")m\n";
  plannerLog << "Number of iterations:\t" << (int)(req.numIterations) <<"\n";
#ifdef USE_FIXEDWING_MODEL
  plannerLog << "Speed:\t\t\t" << g_speed <<"m/s\n";
  plannerLog << "Rmin:\t\t\t" << sys_t::r_min <<"m\n";
#else
  plannerLog << "Linear speed:\t\t" << g_speed <<"m/s\n";
  plannerLog << "Yaw rate:\t\t" << g_maxAngularSpeed <<"rad/s\n";
#endif
  plannerLog << "Field of view [h,v]:\t[" << g_camAngleHorizontal << ", " << g_camAngleVertical << "]rad\n";
  plannerLog << "Camera pitch:\t\t" << g_camPitch <<"rad\n\n";
  plannerLog.close();

  /* initialize problem setup */
  g_cost = DBL_MAX;
  ROS_INFO("Request received");
#ifdef __TIMING_INFO__
  time_DBS = 0;
  time_RRTS = 0;
  time_RRTS_req = 0;
  time_LKH = 0;
#endif
  time_start = 0;
  problemBoundary.setNumDimensions(3);
  assert(req.spaceCenter.size() == 3 && req.spaceSize.size());
  for(int i=0; i<3;i++)
  {
    problemBoundary.size[i] = req.spaceSize[i];
    problemBoundary.center[i] = req.spaceCenter[i];
  } 
  res_g = &res;

  g_max_obs_dim = 0;
  if(req.requiredPoses.size() < 1)
  {
    ROS_ERROR("No start position defined! This is required.");
    std::string pkgPath = ros::package::getPath("koptplanner");
    std::fstream plannerLog;
    plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
    if(!plannerLog.is_open())
      ROS_ERROR("Could not open report.log");
    plannerLog << "-->No start position defined! This is required by the current implementation.\n";
    plannerLog.close();
    koptError = NO_START_POSITION;
    return true;
  }

  timeval time;
  gettimeofday(&time, NULL);
  long millisecStart = time.tv_sec * 1000 + time.tv_usec / 1000;
  time_start = millisecStart;
  ros::Rate visualizerRate(1000);
  bool endPoint = false;

  tri_t::setCamBoundNormals();
  std::vector<tri_t*> tri;
  tri_t::setParam(req.incidenceAngle, req.minDist, req.maxDist); // incidence angle from surface plane
  
  /* startpoint and further strictly required positions */
  for(std::vector<geometry_msgs::Pose>::iterator itFixedPoses = req.requiredPoses.begin(); itFixedPoses != req.requiredPoses.end() && (itFixedPoses != req.requiredPoses.end()-1 || req.requiredPoses.size()==1); itFixedPoses++)
  {
    tri_t * tmp = new tri_t;
    tmp->x1[0] = itFixedPoses->position.x;
    tmp->x1[1] = itFixedPoses->position.y;
    tmp->x1[2] = itFixedPoses->position.z;
    tf::Pose pose;
    tf::poseMsgToTF(*itFixedPoses, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());
#ifdef USE_ROTORCRAFT_MODEL
    tmp->x2[0] = yaw_angle;
    tmp->x2[1] = 0.0;
    tmp->x2[2] = 0.0;
#elif defined USE_FIXEDWING_MODEL
    tmp->x2[0] = 0.0;
    tmp->x2[1] = 0.0;
    tmp->x2[2] = yaw_angle;
#endif
    tmp->Fixpoint = true;
    tri.push_back(tmp);
  }

  /* obstacles */
  std::vector<geometry_msgs::Pose>::iterator itPose = req.obstaclesPoses.begin();
  std::vector<int>::iterator itOccupancy = req.obstacleIntransparancy.begin();
  for(std::vector<shape_msgs::SolidPrimitive>::iterator it = req.obstacles.begin(); it != req.obstacles.end() && itPose != req.obstaclesPoses.end() && itOccupancy != req.obstacleIntransparancy.end(); it++, itPose++, itOccupancy++)
  {
    if(it->type != shape_msgs::SolidPrimitive::BOX)
    {
      ROS_ERROR("Invalid obstacle shape: Only BOX is supported at this time.");
      std::string pkgPath = ros::package::getPath("koptplanner");
      std::fstream plannerLog;
      plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
      if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
      plannerLog << "-->Invalid obstacle shape: Only BOX is supported at this time.\n";
      plannerLog.close();
      koptError = INVALID_OBSTACLE_SHAPE;
      return true;
    }
    reg_t * obs = new reg_t;
    obs->setNumDimensions(3);
    obs->occupied = *itOccupancy;
    obs->center[0] = itPose->position.x;
    obs->center[1] = itPose->position.y;
    obs->center[2] = itPose->position.z;
    assert(it->type == shape_msgs::SolidPrimitive::BOX);
    obs->size[0] = it->dimensions[shape_msgs::SolidPrimitive::BOX_X];
    obs->size[1] = it->dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    obs->size[2] = it->dimensions[shape_msgs::SolidPrimitive::BOX_Z];
    if(g_max_obs_dim<obs->size[0])
      g_max_obs_dim=obs->size[0];
    if(g_max_obs_dim<obs->size[1])
      g_max_obs_dim=obs->size[1];
    if(g_max_obs_dim<obs->size[2])
      g_max_obs_dim=obs->size[2];
    sys_t::obstacles.push_back(obs);
  }
  /* mesh */
  for(std::vector<geometry_msgs::Polygon>::iterator it = req.inspectionArea.begin(); it != req.inspectionArea.end(); it++)
  {
    tri_t* tmp = new tri_t;
    tmp->x1[0] = (*it).points[0].x;
    tmp->x1[1] = (*it).points[0].y;
    tmp->x1[2] = (*it).points[0].z;

    tmp->x2[0] = (*it).points[1].x;
    tmp->x2[1] = (*it).points[1].y;
    tmp->x2[2] = (*it).points[1].z;

    tmp->x3[0] = (*it).points[2].x;
    tmp->x3[1] = (*it).points[2].y;
    tmp->x3[2] = (*it).points[2].z;

    Vector3f a = ((tmp->x2-tmp->x1).cross(tmp->x3-tmp->x2))/2;

    if(a.norm() == 0.0)
    {
      std::string pkgPath = ros::package::getPath("koptplanner");
      std::fstream plannerLog;
      plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
      if(!plannerLog.is_open())
        ROS_ERROR("Could not open report.log");
      plannerLog << "-->Triangle does not have any area. Normal can not be computed. Omitting!\n";
      plannerLog << "   x1: (" << tmp->x1[0] << ", " << tmp->x1[1] << ", " << tmp->x1[2] << ")\n";
      plannerLog << "   x2: (" << tmp->x2[0] << ", " << tmp->x2[1] << ", " << tmp->x2[2] << ")\n";
      plannerLog << "   x3: (" << tmp->x3[0] << ", " << tmp->x3[1] << ", " << tmp->x3[2] << ")\n";
      plannerLog.close();
    }
    else
    {
      tmp->init();
      tri.push_back(tmp);
    }
  }
  /* end point */
  if(req.requiredPoses.size()>1 && // selecting the same start and end point causes the LKH to fail
      ((req.requiredPoses.end()-1)->position.x != req.requiredPoses[0].position.x ||
      (req.requiredPoses.end()-1)->position.y != req.requiredPoses[0].position.y ||
      (req.requiredPoses.end()-1)->position.z != req.requiredPoses[0].position.z ||
      (req.requiredPoses.end()-1)->orientation.x != req.requiredPoses[0].orientation.x ||
      (req.requiredPoses.end()-1)->orientation.y != req.requiredPoses[0].orientation.y ||
      (req.requiredPoses.end()-1)->orientation.z != req.requiredPoses[0].orientation.z ||
      (req.requiredPoses.end()-1)->orientation.w != req.requiredPoses[0].orientation.w))
  {
    g_closed_tour = false;
    endPoint = true;
    tri_t * tmp = new tri_t;
    tmp->x1[0] = (req.requiredPoses.end()-1)->position.x;
    tmp->x1[1] = (req.requiredPoses.end()-1)->position.y;
    tmp->x1[2] = (req.requiredPoses.end()-1)->position.z;
    tf::Pose pose;
    tf::poseMsgToTF(*(req.requiredPoses.end()-1), pose);
    double yaw_angle = tf::getYaw(pose.getRotation());
#ifdef USE_ROTORCRAFT_MODEL
    tmp->x2[0] = yaw_angle;
    tmp->x2[1] = 0.0;
    tmp->x2[2] = 0.0;
#elif defined USE_FIXEDWING_MODEL
    tmp->x2[0] = 0.0;
    tmp->x2[1] = 0.0;
    tmp->x2[2] = yaw_angle;
#endif
    tmp->Fixpoint = true;
    tri.push_back(tmp);
  }
  
  std::fstream file;
  file.open(g_tourlength.c_str(), std::ios::out);
  if(file.is_open())
  {
    file << "length=[";
    file.close();
  }
  else
  {
    ROS_WARN("tourlength file not found");
  }

  maxID = tri.size();
  if(maxID<4)
  {
    ROS_ERROR("Too few inspection areas specified. A minium number of 4 is required, where only %i are given.", maxID);
    std::string pkgPath = ros::package::getPath("koptplanner");
    std::fstream plannerLog;
    plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
    if(!plannerLog.is_open())
      ROS_ERROR("Could not open report.log");
    plannerLog << "-->Too few inspection areas specified. A minium number of 4 is required, where only " << maxID << " are given.\n";
    plannerLog.close();
    koptError = TOO_FEW_INSPECTION_AREAS;
    return true;
  }
  if(VP)
  {
    delete[] VP;
  }
#ifdef USE_FIXEDWING_MODEL
  VP = new StateVector[2*maxID];
  /* load lookup table */
  std::fstream lookupFile;
  lookupFile.open((pkgPath+"/lookupTable/lookupTable50x50.txt").c_str(), std::ios::in);
  lookupTable = new double*[LOOKUPTABLE_SIZE];
  for (int i = 0; i<LOOKUPTABLE_SIZE; i++)
  {
    lookupTable[i] = new double[LOOKUPTABLE_SIZE];
    for (int j = 0; j<LOOKUPTABLE_SIZE; j++)
    {
      lookupFile >> lookupTable[i][j];
    }
  }
#else
  VP = new StateVector[maxID];
#endif
  if(reinitRRTs==NULL)
    reinitRRTs = new int[maxID];
  for(int q = 0; q<maxID; q++)
    reinitRRTs[q] = 1;
  int koptPlannerIteration = 0;
  /* run planner for numIterations */
  while(!ros::isShuttingDown() && koptPlannerIteration < req.numIterations)
  {
    ROS_INFO("STARTING ITERATION %i", koptPlannerIteration);
    plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
    if(!plannerLog.is_open())
      ROS_ERROR("Could not open report.log");
    plannerLog << "\n====================================\n";
    plannerLog << "====== Starting Iteration  " << (koptPlannerIteration>9 ? "" : " ") << koptPlannerIteration << " ======\n";
    plannerLog << "====================================\n";
    plannerLog.close();

    /* -------- ART -- GALLERY -------- */
    for(int i = 0; i < maxID; i++)
    {
      StateVector * s1 = NULL;
      StateVector * s2 = NULL;
      /* find neighbour viewpoints if a tour exists already (koptPlannerIteration>0) */
      if(koptPlannerIteration>0)
      {
        s1 = new StateVector;
        s2 = new StateVector;
        int kpred = 0;
        while(kpred<maxID-1&&PTPPlanner::Tour_[kpred]-1!=i)
          kpred++;
        int ksucc = kpred;
        if(kpred<=0)
        {
          kpred=maxID-1;
        }
        else if(kpred>=maxID-1)
        {
          ksucc=0;
        }
        else
        {
          kpred--;
          ksucc++;
        }
        int Npred = PTPPlanner::Tour_[kpred]-1;
        int Nsucc = PTPPlanner::Tour_[ksucc]-1;
        assert(Npred<=maxID);
        assert(Nsucc<=maxID);
        *s1 = plannerArray[Npred]->rrts_.getLastVisible(plannerArray[i]->stateVec, tri[i]);
#ifndef USE_FIXEDWING_MODEL
        int lim = 0;
        double smoothing_param = std::min(req.numIterations/2,25);
        double neighbour_steps = std::min(2.0, 1.0+50.0/req.numIterations);
        int maxWidth = (int)std::min(neighbour_steps*(double)(req.numIterations-smoothing_param-koptPlannerIteration), (double)(maxID)/2.0);

        while((*s1)[0] == plannerArray[Npred]->stateVec[0] && (*s1)[1] == plannerArray[Npred]->stateVec[1] && (*s1)[2] == plannerArray[Npred]->stateVec[2] && (lim++)<maxWidth)
        {
          if(kpred == 0)
            kpred = maxID-1;
          else
            kpred--;
          int NpredOld = Npred;
          Npred = PTPPlanner::Tour_[kpred]-1;
          *s1 = plannerArray[Npred]->rrts_.getLastVisible(plannerArray[NpredOld]->stateVec, tri[i]);
        }
#endif
        *s2 = plannerArray[Nsucc]->rrts_.getLastVisible(plannerArray[i]->stateVec, tri[i]);
#ifndef USE_FIXEDWING_MODEL
        lim = 0;
        while((*s2)[0] == plannerArray[Nsucc]->stateVec[0] && (*s2)[1] == plannerArray[Nsucc]->stateVec[1] && (*s2)[2] == plannerArray[Nsucc]->stateVec[2] && (lim++)<maxWidth)
        {
          if(ksucc == maxID-1)
            ksucc = 0;
          else
            ksucc++;
          int NsuccOld = Nsucc;
          Nsucc = PTPPlanner::Tour_[ksucc]-1;
          *s2 = plannerArray[Nsucc]->rrts_.getLastVisible(plannerArray[NsuccOld]->stateVec, tri[i]);
        }
#endif
      }
      /* sample viewpoint */
      StateVector VPtmp = tri[i]->dualBarrierSampling(s1,s2,&VP[i]);
      if(koptPlannerIteration == 0 || vp_tol<(VPtmp - VP[i]).norm())
      {
        VP[i] = VPtmp;
        reinitRRTs[i] = 1; // delete (if existing) and reinitialize rrt* tree
      }
      else
        reinitRRTs[i] = 2; // only add iterations to the rrt* tree and don't adapt the viewpoint pose
      delete s1;
      s1 = NULL;
      delete s2;
      s2 = NULL;

      /* display sampled viewpoint in rviz */
      visualization_msgs::Marker point;
      point.header.frame_id = "/kopt_frame";
      point.header.stamp = ros::Time::now();
      point.id = i;
      point.ns = "Viewpoints";
      point.type = visualization_msgs::Marker::ARROW;
      point.pose.position.x = VP[i][0];
      point.pose.position.y = VP[i][1];
      point.pose.position.z = VP[i][2];

#if DIMENSIONALITY>4
      tf::Quaternion q = tf::createQuaternionFromRPY(VP[i][3],VP[i][4],VP[i][5]);
#else
      tf::Quaternion q = tf::createQuaternionFromRPY(0,0,VP[i][3]);
#endif
      point.pose.orientation.x = q.x();
      point.pose.orientation.y = q.y();
      point.pose.orientation.z = q.z();
      point.pose.orientation.w = q.w();

      double scaleVP = sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0;
      point.scale.x = scaleVP;
      point.scale.y = scaleVP/25.0;
      point.scale.z = scaleVP/25.0;
      point.color.r = 0.8f;
      point.color.g = 0.5f;
      point.color.b = 0.0f;
      point.color.a = 0.7;
      point.lifetime = ros::Duration();
      viewpoint_pub.publish(point);
    }
    tri_t::initialized = true;
    
    if(koptError != SUCCESSFUL)
    {
      ROS_ERROR("Error occured! ID: %i", koptError);
      return koptError == SUCCESSFUL;
    }
    /* ------------- TSP -------------- */
    double ** vals = new double* [maxID];
    for(int i = 0; i<maxID; i++)
    {
      vals[i] = new double[3];
      vals[i][0] = VP[i][0]*g_scale;
      vals[i][1] = VP[i][1]*g_scale;
      vals[i][2] = VP[i][2]*g_scale;
    }
    /* use provided interface of the TSP solver */
    std::string params = "MOVE_TYPE=5\n";
    params += "PRECISION=1\n";
    params += "PATCHING_C=3\n";
    params += "PATCHING_A=2\n";
    params += "RUNS=1\n";
    params += "TIME_LIMIT=5\n";
    params += "TRACE_LEVEL=0\n";
    params += "OUTPUT_TOUR_FILE=tempTour.txt\n";
    params += "EOF";

    std::string prob = "NAME:inspection\n";
#ifdef USE_FIXEDWING_MODEL
    prob += "TYPE:ATSP\n";
    prob += "EDGE_WEIGHT_FORMAT:FULL_MATRIX\n";
    prob += "EDGE_WEIGHT_TYPE:RRTFIXEDWING_3D\n";
#else
    prob += "TYPE:TSP\n";
    prob += "EDGE_WEIGHT_TYPE:RRT_3D\n";
#endif
    std::stringstream ss; ss<<maxID;
    prob += "DIMENSION:"+ss.str()+"\n";
    if(endPoint)
      prob += "FIXED_EDGES_SECTION\n"+ss.str()+" 1\n-1\n";
    prob += "NODE_COORD_SECTION\n";
    prob += "EOF";
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_LKH -= time.tv_sec * 1000000 + time.tv_usec;
#endif

    size_t length = params.length();
    char * par;
    assert(par = (char*) malloc(length+10));
    strcpy(par, params.c_str());
    length = prob.length();
    char * pro;
    assert(pro = (char*) malloc(length+10));
    strcpy(pro, prob.c_str());
    /* call TSP solver */
    LKHmainFunction(maxID,vals,par,pro);
#ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_LKH += time.tv_sec * 1000000 + time.tv_usec;
#endif
    for(int i = 0; i<maxID; i++)
    {
      delete[] vals[i];
    }
    delete[] vals;
    Gain23Static = 1;
    GreedyTourmark = 1;
    SFCTourRank = 1;
    ReadPenaltiesStatic = 1;

    koptPlannerIteration++;
  } // while

  res.cost = g_cost;
  file.open(g_tourlength.c_str(), std::ios::app | std::ios::out);
  if(file.is_open())
  {
    file << "];\n";
    gettimeofday(&time, NULL);
    file << "timeEval = " << (int)((long)(time.tv_sec * 1000 + time.tv_usec / 1000) - millisecStart) << ";\n";
#ifdef __TIMING_INFO__
    file << "timeLKH = " << (int)(time_LKH/1000) << ";\n";
    file << "timeRRTs = " << (int)(time_RRTS/1000) << ";\n";
    file << "timeDistEval = " << (int)(time_RRTS_req/1000) << ";\n";
    file << "timeDBS = " << (int)(time_DBS/1000) << ";\n";
#endif
    file.close();
  }

  gettimeofday(&time, NULL);
  ROS_INFO("Calculation time was:\t\t\t%i ms", (int)((long)(time.tv_sec * 1000 + time.tv_usec / 1000) - millisecStart));
#ifdef __TIMING_INFO__
  ROS_INFO("LKH time consumption:\t\t\t%i ms", (int)(time_LKH/1000));
  ROS_INFO("Initial RRT* time consumption:\t\t%i ms", (int)(time_RRTS/1000));
  ROS_INFO("Distance evaluation time:\t\t%i ms", (int)(time_RRTS_req/1000));
  ROS_INFO("Viewpoint sampling time consumption:\t%i ms", (int)(time_DBS/1000));
#endif

  if(koptError == SUCCESSFUL)
    ROS_INFO("sending back response");
  else
    ROS_ERROR("Error occured! ID: %i", koptError);

  // tidy up
  for(int i = 0; i<maxID; i++)
    delete plannerArray[i];
  delete[] plannerArray;
  delete[] reinitRRTs;
  reinitRRTs = NULL;
  for(typename std::vector<tri_t*>::iterator it = tri.begin(); it != tri.end(); it++)
    delete (*it);
  tri.clear();
  for(typename std::list<reg_t*>::iterator it = sys_t::obstacles.begin(); it != sys_t::obstacles.end(); it++)
    delete (*it);
  sys_t::obstacles.clear();
  delete[] VP;
  VP = NULL;
  maxID = 0;
  if(lookupTable)
  {
    for(int i = 0; i < LOOKUPTABLE_SIZE; i++)
      delete[] lookupTable[i];
    delete[] lookupTable;
    lookupTable = NULL;
  }
  plannerArrayBool = false;
  tri_t::initialized = false;

  return koptError == SUCCESSFUL;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "koptplanner");
  ros::NodeHandle n;
  reinitRRTs = NULL;
  maxID = 0;
  VP = NULL;
  lookupTable = NULL;
  plannerArrayBool = false;
  g_closed_tour = true;

  // static function variable reset variables:
  Gain23Static = 0;
  GreedyTourmark = 0;
  SFCTourRank = 0;
  ReadPenaltiesStatic = 0;

  marker_pub = n.advertise<nav_msgs::Path>("visualization_marker", 1);
  viewpoint_pub = n.advertise<visualization_msgs::Marker>("viewpoint_marker", 1);

  ros::ServiceServer service = n.advertiseService("inspectionPath", plan);
  ROS_INFO("Service started");
  ros::spin();

  return 0;
}
