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

Shell #1
```sh
roslaunch koptplanner kopt.launch
```
Shell #2
```sh
rosrun request request 
```


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


Detailed Documentation
---------------------------
Detailed documentation may be found at: StructuralInspectionPlanner/koptplanner/doc/doc.pdf


References:
---------------------------
1. A. Bircher, K. Alexis, M. Burri, P. Oettershagen, S. Omari, T. Mantel and R. Siegwart, “Structural inspection path planning via iterative viewpoint resampling with application to aerial robotics,” in Robotics and Automation (ICRA), 2014 IEEE International Conference on, May 2015, (accepted).

If you use this software in a scientific publication, please cite the following paper:
```
@INPROCEEDINGS{bircher15inspection, 
author = "{A. Bircher, K. Alexis, M. Burri, P. Oettershagen, S. Omari, T. Mantel and R. Siegwart}",
booktitle = {Robotics and Automation (ICRA), 2014 IEEE International Conference on}, 
title={Structural Inspection Path Planning via Iterative Viewpoint Resampling with Application to Aerial Robotics},
year={2015}, 
month={May}, 
note = {(accepted)},
}
```


Credits:
---------------------------
This algorithm was developed by [Andreas Bircher](mailto:bircher@gmx.ch) with the help and support of the members of the [Autonomous Systems Lab](http://www.asl.ethz.ch). The work was supported by the European Commission-funded projects [AEROWORKS](http://www.aeroworks2020.eu/) and [ICARUS](http://www.fp7-icarus.eu/). 


Contact:
---------------------------
You can contact us for any question or remark:
* [Andreas Bircher](mailto:bircher@gmx.ch)
* [Kostas Alexis](konstantinos.alexis@mavt.ethz.ch)
