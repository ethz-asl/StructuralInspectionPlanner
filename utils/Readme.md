# StructuralInspectionPlanner Utilities

This section contains a set of useful utilities that can help either to interface the planner with widespread autopilots or to work on further developments.

Export to PX4/Pixhawk
-------

**Export to PX4/Pixhawk**

Within *utils/ExportToPX4* a python script called *SIP2PX4.py* reads a CSV file that contains the results of the StructuralInspectionPlanner and writes a PX4/Pixhawk mission file. To run the script:

    $ python SIP2PX4.py -i <inputfile.csv> -o <outputfile.txt>

**KML Path Export to PX4/Pixhawk**

Within *utils/ExportToPX4* a python script called *KML2PX4.py* reads a KML file that contains a Path and writes a PX4/Pixhawk mission file. An example script execution would be: 

    $ python KML2PX4.py --file=ExampleKML.kml --output=KMLmissionPX4.txt --radius=30  --auto=1 --sparsify=1 --height=100

note: set height=-1 to retrieve altitude reference form the KML file

Export to DJI drones
-------

Within *utils/ExportToDJI* a python script called *SIP2DJI.py* reads a CSV file that contains the results of the StructuralInspectionPlanner and writes a DJI drone mission file. To run the script:

    $ python SIP2DJI.py -i <inputfile.csv> -o <outputfile.awm>
    
Export to RotorS simulator
-------

Within *utils/ExportToRotorS* a python script called *SIP2RotorS.py* reads a CSV file that contains the results of the StructuralInspectionPlanner and writes a waypoint file. Set maximum velocities and sampling time inside the script. To run the script:

    $ python SIP2RotorS.py -i <inputfile.csv> -o <outputfile.txt>

After running the StructuralInspectionPlanner, find the CSV file at *koptplanner/data/latestPath.csv*.

Path to Trajectory
-------

Within *utils/PathToTrajectory* a python script called *PATH2TRAJ.py* reads a CSV file that contains the results of the StructuralInspectionPlanner and writes an output file that corresponds to a timed-trajectory in case the employed controller requires such an input. The timed-trajectory is based on closed-loop linear simulation and only refers to holonomic systems. To run the script:

    $ python py_compile -O -m PATH2TRAJ.py # to enable basic optimization flags
    $ python PATH2TRAJ.pyo -i <inputfile> -o <outputfile>


Tools for Development
-------

Apart from releasing the source code of the StructuralInspectionPlanner, a set of tools to allow easier further development is provided and will be further populated in the future. This set of tools so far includes:

**LKH TSP Solver Python Interface**

Within *utils/Tools/LKH_Python_Interface* a python script called *InvokeLKH.py* interfaces a compiled version of the LKH TSP Solver and exports the solution in the form of a file. To run the script: 

    $ python InvokeLKH.py

**Airplane 2.5D script**

To find a path that connects two given configurations [x0,y0,z0,yaw0] and [x1,y1,z1,yaw1] using a vehicle that has a minimum turning radius constraint and a max ascending/descending rate using a 2.5D approximation, use may use the tool found in *utils/Tools/Airplane2p5D*. To run the command:

    $ python Airplane2p5D.py --output=<output_file.txt>
