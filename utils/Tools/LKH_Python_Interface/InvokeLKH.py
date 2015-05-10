#		__InvokeLKH__
#		Interface the TSP LKH Solver
# 
# 		This script is a simple python interface to a compiled 
#		version of the LKH TSP Solver. It requires that the
#		solver is compiled at the given directories.
#
#
#		Example Syntax:
#		python InvokeLKH.py
#
#	This script is part of the "utils" section of the StructuralInspectionPlanner
#	Toolbox. A set of elementary components are released together with this 
#	path-planning toolbox in order to make further developments easier. 
# 
import os
import math
import numpy as np

# Change with the Cost Matrix of your problem or 
# consider using it as an argument
CostMatrix = np.ones((10,10))*100

fname_tsp = "test"
user_comment = "a comment by the user"

# Change these directories based on where you have 
# a compiled executable of the LKH TSP Solver
lkh_dir = '/LKH/LKH-2.0.7/'
tsplib_dir = '/TSPLIB/'
lkh_cmd = 'LKH'
pwd= os.path.dirname(os.path.abspath(__file__))


def writeTSPLIBfile_FE(fname_tsp,CostMatrix,user_comment):

	dims_tsp = len(CostMatrix)
	name_line = 'NAME : ' + fname_tsp + '\n'
	type_line = 'TYPE: TSP' + '\n'
	comment_line = 'COMMENT : ' + user_comment + '\n'
	tsp_line = 'TYPE : ' + 'TSP' + '\n'
	dimension_line = 'DIMENSION : ' + str(dims_tsp) + '\n'
	edge_weight_type_line = 'EDGE_WEIGHT_TYPE : ' + 'EXPLICIT' + '\n' # explicit only
	edge_weight_format_line = 'EDGE_WEIGHT_FORMAT: ' + 'FULL_MATRIX' + '\n'
	display_data_type_line ='DISPLAY_DATA_TYPE: ' + 'NO_DISPLAY' + '\n' # 'NO_DISPLAY'
	edge_weight_section_line = 'EDGE_WEIGHT_SECTION' + '\n'
	eof_line = 'EOF\n'
	Cost_Matrix_STRline = []
	for i in range(0,dims_tsp):
		cost_matrix_strline = ''
		for j in range(0,dims_tsp-1):
			cost_matrix_strline = cost_matrix_strline + str(int(CostMatrix[i][j])) + ' '

		j = dims_tsp-1
		cost_matrix_strline = cost_matrix_strline + str(int(CostMatrix[i][j]))
		cost_matrix_strline = cost_matrix_strline + '\n'
		Cost_Matrix_STRline.append(cost_matrix_strline)
	
	fileID = open((pwd + tsplib_dir + fname_tsp + '.tsp'), "w")
	print name_line
	fileID.write(name_line)
	fileID.write(comment_line)
	fileID.write(tsp_line)
	fileID.write(dimension_line)
	fileID.write(edge_weight_type_line)
	fileID.write(edge_weight_format_line)
	fileID.write(edge_weight_section_line)
	for i in range(0,len(Cost_Matrix_STRline)):
		fileID.write(Cost_Matrix_STRline[i])

	fileID.write(eof_line)
	fileID.close()

	fileID2 = open((pwd + tsplib_dir + fname_tsp + '.par'), "w")

	problem_file_line = 'PROBLEM_FILE = ' + pwd + tsplib_dir + fname_tsp + '.tsp' + '\n' # remove pwd + tsplib_dir
	optimum_line = 'OPTIMUM 378032' + '\n'
	move_type_line = 'MOVE_TYPE = 5' + '\n'
	patching_c_line = 'PATCHING_C = 3' + '\n'
	patching_a_line = 'PATCHING_A = 2' + '\n'
	runs_line = 'RUNS = 10' + '\n'
	tour_file_line = 'TOUR_FILE = ' + fname_tsp + '.txt' + '\n'

	fileID2.write(problem_file_line)
	fileID2.write(optimum_line)
	fileID2.write(move_type_line)
	fileID2.write(patching_c_line)
	fileID2.write(patching_a_line)
	fileID2.write(runs_line)
	fileID2.write(tour_file_line)
	fileID2.close()
	return fileID, fileID2

def copy_toTSPLIBdir_cmd(fname_basis):
	copy_toTSPLIBdir_cmd = 'cp' + ' ' + pwd + '/' + fname_basis + '.txt' + ' ' +  pwd + tsplib_dir
	os.system(copy_toTSPLIBdir_cmd)

def run_LKHsolver_cmd(fname_basis):
	run_lkh_cmd =  pwd + lkh_dir  + lkh_cmd + ' ' + pwd + tsplib_dir + fname_basis + '.par'
	os.system(run_lkh_cmd)

def rm_solution_file_cmd(fname_basis):
	rm_sol_cmd = 'rm' + ' ' + pwd + '/' + fname_basis + '.txt'
	os.system(rm_sol_cmd) 


def main(): 
	
	[fileID1,fileID2] = writeTSPLIBfile_FE(fname_tsp,CostMatrix,user_comment)
	run_LKHsolver_cmd(fname_tsp)
	copy_toTSPLIBdir_cmd(fname_tsp)
	rm_solution_file_cmd(fname_tsp)


if __name__ == "__main__":
    main()

