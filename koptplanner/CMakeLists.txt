cmake_minimum_required(VERSION 2.8.3)
project(koptplanner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  shape_msgs
  message_generation
  optec
)

find_package(cmake_modules REQUIRED)
find_package(optec REQUIRED)
find_package(Eigen REQUIRED)

set(CMAKE_BUILD_TYPE Release)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  inspection.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  shape_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
CATKIN_DEPENDS message_runtime optec
INCLUDE_DIRS  ${Eigen_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/koptplanner
  include/planner_rrts
  include/LKH-2.0.7/SRC
  include/LKH-2.0.7/SRC/INCLUDE
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(plan
  src/plan.cpp
  src/ptpPlanner.cpp
  src/Rotorcraft.cpp
  src/FixedWing.cpp
  src/system_holonomic.cpp
  include/planner_rrts/kdtree.c
  include/LKH-2.0.7/SRC/LKHmain.c
  include/LKH-2.0.7/SRC/Activate.c
  include/LKH-2.0.7/SRC/AddCandidate.c
  include/LKH-2.0.7/SRC/AddExtraCandidates.c
  include/LKH-2.0.7/SRC/AddTourCandidates.c
  include/LKH-2.0.7/SRC/AdjustCandidateSet.c
  include/LKH-2.0.7/SRC/AllocateStructures.c
  include/LKH-2.0.7/SRC/Ascent.c
  include/LKH-2.0.7/SRC/Best2OptMove.c
  include/LKH-2.0.7/SRC/Best3OptMove.c
  include/LKH-2.0.7/SRC/Best4OptMove.c
  include/LKH-2.0.7/SRC/Best5OptMove.c
  include/LKH-2.0.7/SRC/BestKOptMove.c
  include/LKH-2.0.7/SRC/Between.c
  include/LKH-2.0.7/SRC/Between_SL.c
  include/LKH-2.0.7/SRC/Between_SSL.c
  include/LKH-2.0.7/SRC/BridgeGain.c
  include/LKH-2.0.7/SRC/BuildKDTree.c
  include/LKH-2.0.7/SRC/C.c
  include/LKH-2.0.7/SRC/CandidateReport.c
  include/LKH-2.0.7/SRC/ChooseInitialTour.c
  include/LKH-2.0.7/SRC/Connect.c
  include/LKH-2.0.7/SRC/CreateCandidateSet.c
  include/LKH-2.0.7/SRC/CreateDelaunayCandidateSet.c
  include/LKH-2.0.7/SRC/CreateQuadrantCandidateSet.c
  include/LKH-2.0.7/SRC/Delaunay.c
  include/LKH-2.0.7/SRC/Distance.c
  include/LKH-2.0.7/SRC/Distance_SPECIAL.c
  include/LKH-2.0.7/SRC/eprintf.c
  include/LKH-2.0.7/SRC/ERXT.c
  include/LKH-2.0.7/SRC/Excludable.c
  include/LKH-2.0.7/SRC/Exclude.c
  include/LKH-2.0.7/SRC/FindTour.c
  include/LKH-2.0.7/SRC/Flip.c
  include/LKH-2.0.7/SRC/Flip_SL.c
  include/LKH-2.0.7/SRC/Flip_SSL.c
  include/LKH-2.0.7/SRC/Forbidden.c
  include/LKH-2.0.7/SRC/FreeStructures.c
  include/LKH-2.0.7/SRC/fscanint.c
  include/LKH-2.0.7/SRC/Gain23.c
  include/LKH-2.0.7/SRC/GenerateCandidates.c
  include/LKH-2.0.7/SRC/Genetic.c
  include/LKH-2.0.7/SRC/GeoConversion.c
  include/LKH-2.0.7/SRC/GetTime.c
  include/LKH-2.0.7/SRC/GreedyTour.c
  include/LKH-2.0.7/SRC/Hashing.c
  include/LKH-2.0.7/SRC/Heap.c
  include/LKH-2.0.7/SRC/IsCandidate.c
  include/LKH-2.0.7/SRC/IsCommonEdge.c
  include/LKH-2.0.7/SRC/IsPossibleCandidate.c
  include/LKH-2.0.7/SRC/KSwapKick.c
  include/LKH-2.0.7/SRC/LinKernighan.c
  include/LKH-2.0.7/SRC/Make2OptMove.c
  include/LKH-2.0.7/SRC/Make3OptMove.c
  include/LKH-2.0.7/SRC/Make4OptMove.c
  include/LKH-2.0.7/SRC/Make5OptMove.c
  include/LKH-2.0.7/SRC/MakeKOptMove.c
  include/LKH-2.0.7/SRC/MergeBetterTourWithBestTour.c
  include/LKH-2.0.7/SRC/MergeWithTour.c
  include/LKH-2.0.7/SRC/Minimum1TreeCost.c
  include/LKH-2.0.7/SRC/MinimumSpanningTree.c
  include/LKH-2.0.7/SRC/NormalizeNodeList.c
  include/LKH-2.0.7/SRC/NormalizeSegmentList.c
  include/LKH-2.0.7/SRC/OrderCandidateSet.c
  include/LKH-2.0.7/SRC/PatchCycles.c
  include/LKH-2.0.7/SRC/printff.c
  include/LKH-2.0.7/SRC/PrintParameters.c
  include/LKH-2.0.7/SRC/qsort.c
  include/LKH-2.0.7/SRC/Random.c
  include/LKH-2.0.7/SRC/ReadCandidates.c
  include/LKH-2.0.7/SRC/ReadLine.c
  include/LKH-2.0.7/SRC/ReadParametersInsp.c
  include/LKH-2.0.7/SRC/ReadPenalties.c
  include/LKH-2.0.7/SRC/ReadProblemInsp.c
  include/LKH-2.0.7/SRC/RecordBestTour.c
  include/LKH-2.0.7/SRC/RecordBetterTour.c
  include/LKH-2.0.7/SRC/RemoveFirstActive.c
  include/LKH-2.0.7/SRC/ResetCandidateSet.c
  include/LKH-2.0.7/SRC/RestoreTour.c
  include/LKH-2.0.7/SRC/SegmentSize.c
  include/LKH-2.0.7/SRC/Sequence.c
  include/LKH-2.0.7/SRC/SFCTour.c
  include/LKH-2.0.7/SRC/SolveCompressedSubproblem.c
  include/LKH-2.0.7/SRC/SolveDelaunaySubproblems.c
  include/LKH-2.0.7/SRC/SolveKarpSubproblems.c
  include/LKH-2.0.7/SRC/SolveKCenterSubproblems.c
  include/LKH-2.0.7/SRC/SolveKMeansSubproblems.c
  include/LKH-2.0.7/SRC/SolveRoheSubproblems.c
  include/LKH-2.0.7/SRC/SolveSFCSubproblems.c
  include/LKH-2.0.7/SRC/SolveSubproblem.c
  include/LKH-2.0.7/SRC/SolveSubproblemBorderProblems.c
  include/LKH-2.0.7/SRC/SolveTourSegmentSubproblems.c
  include/LKH-2.0.7/SRC/Statistics.c
  include/LKH-2.0.7/SRC/StoreTour.c
  include/LKH-2.0.7/SRC/SymmetrizeCandidateSet.c
  include/LKH-2.0.7/SRC/TrimCandidateSet.c
  include/LKH-2.0.7/SRC/WriteCandidates.c
  include/LKH-2.0.7/SRC/WritePenalties.c
  include/LKH-2.0.7/SRC/WriteTour.c
)

## Specify libraries to link a library or executable target against
target_link_libraries(plan
  ${optec_LIBRARIES}
  ${catkin_LIBRARIES}
)
