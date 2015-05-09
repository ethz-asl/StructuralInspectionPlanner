#		__Airplane2p5D__
# 		2.5D Airplane Point to Point Connections
#		A Python script for simplified Aircraft Kinematics
# 
# 		Computes Point - to - Point Aircraft connections based on
# 		2D dubins curves for the XY-plane and constrained linear 
# 		interpolation for the altitude component.
#
# 		For the dubins curve solution, the code uses the dubins 
# 		package available at: https://github.com/AndrewWalker/pydubins
#
#		Example Syntax:
#		python Airplane2p5D.py --output=airplane_solution.txt
#
#	This script is part of the "utils" section of the StructuralInspectionPlanner
#	Toolbox. A set of elementary components are released together with this 
#	path-planning toolbox in order to make further developments easier. 
# 

import dubins
import numpy as np
import math
import matplotlib.pyplot as mpplot
from optparse import OptionParser


class VehicleParameters(object):
	def __init__(self,MinTurnRadius,MaxAscDescRate,V_travel):
		self.MinTurnRadius = MinTurnRadius
		self.MaxAscDescRate = MaxAscDescRate
		self.V_travel = V_travel

class VehicleConfiguration(object):
	def __init__(self,x0,y0,z0,yaw0):
		self.x0 = x0
		self.y0 = y0
		self.z0 = z0
		self.yaw0 = yaw0

class SolverParameters(object):
	def __init__(self,StepSize):
		self.StepSize = StepSize

class VehicleSolution(object):
	def __init__(self,path_3D,yaw):
		self.Path = path_3D
		self.Xvec = path_3D[:,0]
		self.Yvec = path_3D[:,1]
		self.Zvec = path_3D[:,2]
		self.YAWvec = yaw

# 	define the problem parameters
q0 = VehicleConfiguration(0.0, 0.0, 100.0, np.pi/4)
q1 = VehicleConfiguration(0.0, 0.0, 50.0, -np.pi/4)
AircraftParameters = VehicleParameters(1.0,10.0,1.0)
DubinsSolver = SolverParameters(0.5) 


def DubinsPathLength(dubinsPath):
    # Compute 2D Dubins Path Length
    sol_2d = np.asarray(dubinsPath)
    [Ns,Np] = sol_2d.shape

    PathLength = 0.0
    for i in range(1,Ns):
    	PathLength = PathLength + np.sqrt( np.power( (sol_2d[i,0]-sol_2d[i-1,0]),2) + np.power( (sol_2d[i,1]-sol_2d[i-1,1]),2) )

    return PathLength

def MaxAscDesc(q0,q1,Ns,MaxAscDescRate,Ts):
	# Max AscDesc connection
	z_vec = np.zeros(Ns)
	z_vec[0] = q0[2]
	for i in range(1,Ns):
		z_vec[i] = z_vec[i-1] - np.sign(q1.z0 - q0.z0)*AircraftParameters.MaxAscDescRate*Ts

	return z_vec

def LinearAscDesc(q0,q1,Ns):
	# Linear Asc Desc Connection of two values
	samples_i = range(Ns)
	z_vec = np.interp(samples_i,np.array([0, Ns]),np.array([q0.z0,q1.z0]))
	return z_vec

def plot3(a,b,c,mark="o",col="r"):
	# mimic matlab plot3
  	from matplotlib import pyplot
  	import pylab
  	from mpl_toolkits.mplot3d import Axes3D
  	pylab.ion()
  	fig = pylab.figure()
  	ax = Axes3D(fig)
  	ax.plot(a, b,c,color=col,marker=mark)
  	fig.show()

def main(): 
	parser = OptionParser()                                                               
	parser.add_option("-d", "--output", dest = "pathfile", 
                   help = "Path output file", 
                   metavar = "FILE")
	(options, args) = parser.parse_args()

	# 2D Dubins Path Part
	q0_2d = np.array([q0.x0, q0.y0, q0.yaw0])
	q1_2d = np.array([q1.x0, q1.y0, q1.yaw0])
	qs, _ = dubins.path_sample(q0_2d, q1_2d, AircraftParameters.MinTurnRadius, DubinsSolver.StepSize)
	sol_2d = np.asarray(qs); [Ns,Ndim] = sol_2d.shape;

	AirplaneSolution = VehicleSolution(np.zeros((Ns,3)),np.zeros((Ns,1)))
	AirplaneSolution.Path[:,0] = sol_2d[:,0]; AirplaneSolution.Path[:,1] = sol_2d[:,1]; AirplaneSolution.YAWvec = sol_2d[:,2]; 
	PathLength = DubinsPathLength(sol_2d)
	TimeToDest = PathLength/AircraftParameters.V_travel

	# altitude path
	RateDes = abs(q0.z0-q1.z0)/TimeToDest
	if RateDes > AircraftParameters.MaxAscDescRate:
		# the system will not arrive to the desired altitude
		# it will rather ascend/descend with the maximum rate
		AirplaneSolution.Path[:,2] = MaxAscDesc(q0,q1,Ns,AircraftParameters.MaxAscDescRate,(PathLength/Ns)/AircraftParameters.V_travel)
	else:
		AirplaneSolution.Path[:,2] = LinearAscDesc(q0,q1,Ns)


	# save to file
	solution_mat = np.zeros((Ns,4))
	solution_mat[:,0] = AirplaneSolution.Path[:,0]; solution_mat[:,1] = AirplaneSolution.Path[:,1]; 
	solution_mat[:,2] = AirplaneSolution.Path[:,2]; solution_mat[:,3] = AirplaneSolution.YAWvec; 
	np.savetxt(options.pathfile,solution_mat)
	# plot option
	plot3(AirplaneSolution.Path[:,0],AirplaneSolution.Path[:,1],AirplaneSolution.Path[:,2],'o','g')
	raw_input()

if __name__ == "__main__":
    main()
