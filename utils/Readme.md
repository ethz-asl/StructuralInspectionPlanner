# StructuralInspectionPlanner Utilities

This section contains useful utilities that help interfacing the paths computed by this toolbox with widely used autopilots or other software. It will be constantly updated.

**Export to PX4/Pixhawk**

Within *utils/ExportToPX4* a python script called *SIP2PX4.py* reads a CSV file that contains the results of the StructuralInspectionPlanner and writes a PX4/Pixhawk mission file. After execution of the path planner this file is located in the folder *koptplanner/data*. To run the script:

    $ python SIP2PX4.py -i <inputfile> -o <outputfile>
