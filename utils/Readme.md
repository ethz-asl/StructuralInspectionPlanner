# StructuralInspectionPlanner Utilities

This section contains a set of useful utilities that can help either to interface the planner with widespread autopilots or to work on further developments.

Export to PX4/Pixhawk
-------

**Export to PX4/Pixhawk**

Within *utils/ExportToPX4* a python script called *SIP2PX4.py* reads a CSV file that contains the results of the StructuralInspectionPlanner and writes a PX4/Pixhawk mission file. To run the script:

    $ python SIP2PX4.py -i <inputfile> -o <outputfile>

**KML Path Export to PX4/Pixhawk**

Within *utils/ExportToPX4* a python script called *KML2PX4.py* reads a KML file that contains a Path and writes a PX4/Pixhawk mission file. An example script execution would be: 

    $ python KML2PX4.py --file=ExampleKML.kml --output=KMLmissionPX4.txt --radius=30  --auto=1 --sparsify=1 --height=100

note: set height=-1 to retrieve altitude reference form the KML file

Tools for Development
-------

Apart from releasing the source code of the StructuralInspectionPlanner, a set of tools to allow easier further development is provided and will be further populated in the future. This set of tools so far includes:

**LKH TSP Solver Python Interface**

Within *utils/Tools/LKH_Python_Interface* a python script called *InvokeLKH.py* interfaces a compiled version of the LKH TSP Solver and exports the solution in the form of a file. To run the script: 

    $ python InvokeLKH.py

**Airplane 2.5D script**

To find a path that connects two given configurations [x0,y0,z0,yaw0] and [x1,y1,z1,yaw1] using a vehicle that has a minimum turning radius constraint and a max ascending/descending rate using a 2.5D approximation, use may use the tool found in *utils/Tools/Airplane2p5D*. To run the command:

    $ python Airplane2p5D.py --output=<output_file.txt>
