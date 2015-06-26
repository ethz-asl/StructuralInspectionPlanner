StructuralInspectionPlanner
===========================
**Beta version**

The structural inspection path planning algorithm presented in our paper contribution [1] is released as an open-source toolbox. The algorithm assumes a triangular mesh representation of the structure and employs an alternating two-step optimization paradigm to find good viewpoints that together provide full coverage and a connecting path that has low cost. In every iteration, the viewpoints are chosen such that the connection cost is reduced and, subsequently, the tour is optimized. Vehicle and sensor limitations are respected within both steps. Sample implementations are provided for rotorcraft and fixed-wing unmanned aerial robots.

Additional functionality allows exportation of the computed paths to drone mission files. Supported systems are PX4/Pixhawk and DJI drones, as well as the RotorS simulator and will be extended in the future. (Refer to the [*utils*](https://github.com/ethz-asl/StructuralInspectionPlanner/tree/master/utils) section)

Installing the toolbox
---------------------------
To use the toolbox a ROS indigo installation with catkin set-up and the following extra packages are required:

```
libeigen3-dev
ros-indigo-tf
ros-indigo-rviz
ros-indigo-octomap
ros-indigo-octomap-msgs
```

Once these are there, a baseline example on how to get and install the tool is the following:

```sh
$ mkdir catkin_ws_repos # assuming you want to enter a new catkin directory
$ cd catkin_ws_repos # alternatively just cd your normal catkin workspace
$ mkdir src # assuming this folder does not exist
$ cd src/
$ catkin_init_workspace
$ git clone git@github.com:ethz-asl/StructuralInspectionPlanner.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
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

For visualization puprposes, a 3rd terminal has to be launched: 

Shell #3
```sh
rosrun rviz rviz
```

Add the necessary displays:

```sh
‘Path’ on topic ‘visualization_marker’
‘Marker’ on topic ‘viewpoint_marker’
‘Path’ on topic ‘stl_mesh’
‘Marker’ on topic ‘scenario’
```
To display the progress, chose ‘/kopt_frame’ as fixed frame or publish a suitable transform.

Detailed Documentation
---------------------------
Detailed documentation may be found at the [Wiki!](https://github.com/ethz-asl/StructuralInspectionPlanner/wiki)


References:
---------------------------
1. A. Bircher, K. Alexis, M. Burri, P. Oettershagen, S. Omari, T. Mantel and R. Siegwart, “Structural inspection path planning via iterative viewpoint resampling with application to aerial robotics,” in Robotics and Automation (ICRA), 2015 IEEE International Conference on, May 2015, pp. 6423–6430.
2. K. Helsguan, "An effective implementation of the lin-kernighan traveling salesman heuristic", European Journal of Operational Research, vol. 126, no. 1, pp. 106-130, 2000.
3. H.J. Ferreau and A. Potschka and C. Kirches, "qpOASES"
4. S. Karaman and E. Frazzoli, "Sampling-based algorithms for optimal motion planning", International Journal of Robotics Research", vol. 30, no. 7, pp. 846-894, 2011

If you use this software in a scientific publication, please cite the following paper:
```
@INPROCEEDINGS{BABOOMS_ICRA_15, 
author = "{A. Bircher, K. Alexis, M. Burri, P. Oettershagen, S. Omari, T. Mantel and R. Siegwart}",
booktitle = {Robotics and Automation (ICRA), 2015 IEEE International Conference on}, 
title={Structural Inspection Path Planning via Iterative Viewpoint Resampling with Application to Aerial Robotics},
year={2015}, 
month={May}, 
pages={6423-6430}, 
}
```


Credits:
---------------------------
This algorithm was developed by [Andreas Bircher](mailto:bircher@gmx.ch) with the help and support of the members of the [Autonomous Systems Lab](http://www.asl.ethz.ch). The work was supported by the European Commission-funded projects [AEROWORKS](http://www.aeroworks2020.eu/) and [ICARUS](http://www.fp7-icarus.eu/). 


Contact:
---------------------------
You can contact us for any question or remark:
* [Andreas Bircher](mailto:bircher@gmx.ch)
* [Kostas Alexis](mailto:konstantinos.alexis@mavt.ethz.ch)
