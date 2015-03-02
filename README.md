StructuralInspectionPlanner
===========================

A new structural inspection path planning algorithm presented in our paper contribution [1] is released as an open-source toolbox. The algorithm assumes a triangular mesh representation of the structure and employs an alternating two-step optimization paradigm to find good viewpoints that together provide full coverage and a connecting path that has low cost. In every iteration, the viewpoints are chosen such that the connection cost is reduced and, subsequently, the tour is optimized. Vehicle and sensor limitations are respected within both steps. Sample implementations are provided for rotorcraft and fixed-wing unmanned aerial robots. 

Installing the toolbox
---------------------------
To use the toolbox a recent ROS installation with catkin support is required. A baseline example on how to get and install is the following:

```sh
$ mkdir catkin_ws_repos # assuming you want to enter a new catkin directory
$ cd catkin_ws_repos # alternatively just cd your normal catkin workspace
$ catkin_make 
$ source devel/setup.bash
$ mkdir src # assuming this folder does not exist
$ cd src/
$ git clone git@github.com:ethz-asl/StructuralInspectionPlanner.git
$ cd ..
$ catkin_make
```

The build process should then be executed and successfully complete. Subsequently open two separate shells and run the following two commands to execute the baseline demo of the algorithm:

roslaunch koptplanner kopt.launch
rosrun request request 

Detailed Documentation
---------------------------
Detailed documentation may be found at: StructuralInspectionPlanner/koptplanner/doc/doc.pdf

Brief Usage Overview
---------------------------
Information about how to use the planner is given below. Further information on how it works can be found in the manual or in the publication 'Structural Inspection Path Planner via Iterative Viewpoint Resampling with Application to Aerial Robotics', ICRA 2015.

Displaying the planning process in rviz
---------------------------
During planning the current best path and viewpoints are outputted to rviz. The necessary displays are:
‘Path’ on topic ‘visualization_marker’,
‘Marker’ on topic ‘viewpoint_marker’,
‘Path’ on topic ‘stl_mesh’ and 
‘Marker’ on topic ‘scenario’.
To display the progress, chose ‘/kopt_frame’ as fixed frame or publish a suitable transform.

Parameters
---------------------------
Most parameters can be chosen as ros parameter (‘koptParam.yaml’). Additional parameters can be found in the file plan.hpp (e.g. Vehicle type).

Starting the planner
---------------------------
To start the planner execute both commands in a shell:
'roslaunch koptplanner kopt.launch' and
'rosrun request request'

Visualization
---------------------------
Use the supplied MATLAB script ‘inspectionPathVisualization.m’ together with the generated file ‘inspectionScenario.m’.

Further Questions
---------------------------
You can always send me an email (bircher@gmx.ch) for questions. If you have any cool application, please also let me know! :)
