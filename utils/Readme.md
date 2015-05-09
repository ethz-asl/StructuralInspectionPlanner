# StructuralInspectionPlanner Utilities

This section contains useful utilities that help interfacing the paths computed by this toolbox with widely used autopilots or other software. It will be constantly updated.

**SIP Mission Export to PX4/Pixhawk**

Within *utils/ExportToPX4* a python script called *SIP2PX4.py* reads a CSV file that contains the results of the StructuralInspectionPlanner and writes a PX4/Pixhawk mission file. After execution of the path planner this file is located in the folder *koptplanner/data*. To run the script:

    $ python SIP2PX4.py -i <inputfile> -o <outputfile>

**KML Path Export to PX4/Pixhawk**

Within *utils/ExportToPX4* a python script called *KML2PX4.py* reads a KML file that contains a Path and writes a PX4/Pixhawk mission file. An example script execution would be: 

    $ python KML2PX4.py --file=ExampleKML.kml --output=KMLmissionPX4.txt --radius=30  --auto=1 --sparsify=1 --height=100

note: set height=-1 to retrieve altitude reference form the KML file
