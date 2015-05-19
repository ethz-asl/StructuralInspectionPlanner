#       __KML2PX4__
#       KML (.kml) to PX4/Pixhawk Mission File Parsing and Export
# 
#       This script accepts a *.kml file as an input and exports the 
#       corresponding PX4/Pixhawk autopilot mission to be loaded using
#       QGroundControl or other compative software
#
#       More Info on PX4/Pixhawk and QGroundControl:
#       http://pixhawk.org/
#       http://qgroundcontrol.org/
#
#       Example Syntax:
#       python KML2PX4.py --file=ExampleKML.kml --output=KMLmissionPX4.txt 
#                         --radius=30  --auto=1 --sparsify=1 --height=100
#   
#       note: set height=-1 to retrieve altitude reference form the KML file
#
#   This script is part of the "utils" section of the StructuralInspectionPlanner
#   Toolbox. A set of elementary components are released together with this 
#   path-planning toolbox in order to make further developments easier. 
# 

import os
import csv
import getpass
import logging
import numpy as np
import re
from optparse import OptionParser

from BeautifulSoup import BeautifulSoup

class PX4MissionPars(object):
    def __init__(self,AcceptanceRadius,AutoContFlag,SparsifyFactor,HeightRef):
        self.AcceptanceRadius = AcceptanceRadius
        self.AutoContFlag = AutoContFlag
        self.SparsifyFactor = SparsifyFactor
        self.HeightRef = HeightRef

class KMLParser(object):
    """
        KmlParser
    """ 
    def __init__(self, kml_file, px4_file, mission_pars):
        self.kml_file = kml_file
        self.px4_file = px4_file
        self.outputfile = ''
        self.outputdata = []
        self.MissionPars = mission_pars
    
    def ParseKml(self): 
        """
            parse_kml
        """
        count = 0
        try:
            handler = open(self.kml_file).read()
            soup = BeautifulSoup(handler)
            for message in soup.findAll('placemark'):
                locationdata = {}
                coordinates = message.find('coordinates')
                locationdata['geometry'] = '<LineString> %s </LineString>' % (coordinates)
                names = message.findAll('name')
                for name in names:
                    text = name.find(text = True)
                    locationdata['name'] = text
                    coordinates = str(coordinates)
                    coordinates = re.split(', | |\n |\t',coordinates)
                    coordinates = coordinates[4:(len(coordinates)-4)]
                    lla_coords = np.zeros((len(coordinates),3))
                    for i in range(0,len(coordinates)):
                        lla_coords[i] = np.array(coordinates[i].split(','))

            self.outputdata.append(lla_coords)                    
        except IOError as (errno, strerror):
            logging.error("I/O error(%d): %s" %(errno, strerror))

    def WritePX4(self):
        """
            write_px4       
        """ 
        self.outputfile = os.getcwd() + '/' + self.px4_file
        try:
            out = open(self.outputfile,'w')
            print 'Writing output to file ', self.outputfile
            try:

                
                out.write("QGC WPL 120")
                out.write("\r")
                out.write("\r\n")    

                count = 0
                for i in range(0,len(self.outputdata[0]),self.MissionPars.SparsifyFactor):
                    if i == 0:
                        if self.MissionPars.HeightRef == -1:
                            z_ref = self.outputdata[0][i][2]
                        else:
                            z_ref = self.MissionPars.HeightRef

                        out.write("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%.17f\t%.17f\t%.0f\t%i" % (count,1,0,16,0,self.MissionPars.AcceptanceRadius,0,0,self.outputdata[0][i][0],self.outputdata[0][i][1],z_ref,self.MissionPars.AutoContFlag))
                        out.write("\r\r\n")
                    else:
                        if self.MissionPars.HeightRef == -1:
                            z_ref = self.outputdata[0][i][2]
                        else:
                            z_ref = self.MissionPars.HeightRef

                        out.write("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%.17f\t%.17f\t%.0f\t%i" % (count,0,0,16,0,self.MissionPars.AcceptanceRadius,0,0,self.outputdata[0][i][0],self.outputdata[0][i][1],z_ref,self.MissionPars.AutoContFlag))
                        out.write("\r\r\n")
                    count = count + 1  
                
                print 'Output file ', self.outputfile, ' written' 
                
            finally:
                out.close()
        except IOError as (errno, strerror):
            logging.error("I/O error(%d): %s" % (errno, strerror))
        return self.outputfile

def main():
    """
        Main method
    """
    parser = OptionParser()
    parser.add_option("-f", "--file", dest = "kmlfile", 
                    help = "KML file to be parsed", 
                    metavar = "FILE")               
    parser.add_option("-a", "--radius", dest = "AcceptanceRadius",
                    help = "PX4 Mission Acceptance Radius",
                    type = "float")    
    parser.add_option("-b", "--auto", dest = "AutoContFlag",
                    help = "PX4 Mission Auto Continue Flag",
                    type = "int")
    parser.add_option("-c", "--sparsify", dest = "SparsifyFactor",
                    help = "KML 2 PX4 Sparsify Factor",
                    type = "int")   
    parser.add_option("-e", "--height", dest = "HeightRef",
                    help = "Height ref (-1 to get height from KML)",
                    type = "float")                                                       
    parser.add_option("-d", "--output", dest = "px4file", 
                   help = "PX4 Mission output file", 
                   metavar = "FILE")
    
    (options, args) = parser.parse_args()
    if not options.kmlfile:
        print "please type python " \
              "KML2PX4.py --file=[kmlfilename] --output=[px4filename]"     
    elif not options.px4file:
        print "please type python " \
              "KML2PX4.py --file=[kmlfilename] --output=[px4filename]"
    else:
        MissionsPars = PX4MissionPars(options.AcceptanceRadius,options.AutoContFlag,options.SparsifyFactor,options.HeightRef)
        kmlparser = KMLParser(kml_file=options.kmlfile, 
                             px4_file=options.px4file,
                             mission_pars=MissionsPars)               
        kmlparser.ParseKml()
        upload_file = kmlparser.WritePX4()

if __name__ == "__main__":
    main()




 
