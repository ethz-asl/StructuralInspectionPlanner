StructuralInspectionPlanner
===========================

Information about how to use the planner is given below. Further information on how it works can be found in the manual or in the publication 'Structural Inspection Path Planner via Iterative Viewpoint Alternation', ICRA 2015.

Displaying the planning process in rviz
---------------------------
During planning the current best path and viewpoints are outputted to rviz. The necessary displays are:
‘Path’ on topic ‘visualization_marker’
‘Marker’ on topic ‘viewpoint_marker’
‘Path’ on topic ‘stl_mesh’
‘Marker’ on topic ‘scenario’
To display the progress, chose ‘/kopt_frame’ as fixed frame or publish a suitable transform.

Parameters
---------------------------
Most parameters can be chosen as ros parameter (‘koptParam.yaml’). Additional parameters can be found in the file plan.hpp (e.g. Vehicle type).

Starting the planner
---------------------------
To start the planner execute both commands in a shell:
roslaunch koptplanner kopt.launch
rosrun request request

Visualization
---------------------------
Use the supplied MATLAB script ‘inspectionPathVisualization.m’ together with the generated file ‘inspectionScenario.m’.

Further Questions
---------------------------
You can always send me an email (bircher@gmx.ch) for questions. If you have any cool application, please also let me know! :)
