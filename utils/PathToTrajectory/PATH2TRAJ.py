#       __PATH2TRAJ__
#       SIP Path to Trajectory based on simple second-order linear model
#       and an LQ controller with saturated control actions
# 
#       This script accepts an input file with the path and exports a 
#       trajectory based on second-order closed-loop simulation of simplified
#       system dynamics and an LQ-controller. Its purpose is to stand as a
#       simple way to derive timed trajectories from paths produced by 
#       the StructuralInspectionPlanner. The extracted trajectory is by no
#       means optimal and depends heavily on the tuning parameters. However,
#       it provided as a quick way to derived a time trajectory as typically
#       required by several controllers on unmanned aerial vehicles.
#
#       Example Syntax:
#       python py_compile -O -m PATH2TRAJ.py # to enable some of the optimization flags
#       python PATH2TRAJ.pyo -i <inputfile> -o <outputfile> # to run the script
#
#   This script is part of the "utils" section of the StructuralInspectionPlanner
#   Toolbox. A set of elementary components are released together with this 
#   path-planning toolbox in order to make further developments easier. 
# 
from scipy import signal
from scipy import cos, sin, arctan, sqrt, arctan2, radians, degrees
from scipy.linalg import block_diag
import numpy as np
import matplotlib.pyplot as plt
import pydare as dare
import sys, getopt

# LQ tuning matrices : these should be retuned
Q_x = np.zeros((2,2)); Q_x[0,0] = 10; Q_x[1,1] = 2000; 
R_x = np.zeros((1,1)); R_x[0,0] = 2;
Q_y = np.zeros((2,2)); Q_y[0,0] = 10; Q_y[1,1] = 2000; 
R_y = np.zeros((1,1)); R_y[0,0] = 2;
Q_z = np.zeros((2,2)); Q_z[0,0] = 20; Q_z[1,1] = 2000; 
R_z = np.zeros((1,1)); R_z[0,0] = 2;

# max sliding error
max_error_xyz = 2;
# accuracy range specifying when a waypoint has been reached
accuracy_range = .1;
# vehicle parameters
mass = 1.2; max_xy_vel = 2; max_z_vel = 2; maxRollPitch = 15; maxDeltaT = 6;
# simulation parameters
sample_time = 0.01;

x_init_x = np.zeros((2,1)); 
x_init_y = np.zeros((2,1));
x_init_z = np.zeros((2,1));
x_init_yaw = np.zeros((1,1));

class VehicleParameters(object):
    """
        Vehicle Parameters
    """ 
    def __init__(self,mass,max_xy_vel,max_z_vel,maxRollPitch,maxDeltaT):
        self.mass = mass;
        self.xvel_max = max_xy_vel;
        self.yvel_max = max_xy_vel;
        self.zvel_max = max_z_vel;
        self.maxRoll = maxRollPitch;
        self.maxPitch = maxRollPitch;
        self.maxDeltaT = maxDeltaT;

class EnvironmentParameters(object):
    """
        Environment Parameters
    """ 
    def __init__(self,g):
        self.g = g;

class TuningParameters(object):
    """
        Tuning Parameters
    """ 
    def __init__(self,Ts,Qx,Rx,Qy,Ry,Qz,Rz,max_err):
        self.Ts = Ts;
        self.Qx = Qx;
        self.Rx = Rx;
        self.Qy = Qy;
        self.Ry = Ry;
        self.Qz = Qz;
        self.Rz = Rz;
        self.max_err = max_err;

class SimulationParameters(object):
    """
        Simulation Parameters
    """ 
    def __init__(self,Ts,t_vec,u_vec,ref_x,ref_y,ref_z):
        self.Ts = Ts
        self.t_vec = t_vec
        self.u_vec = u_vec
        self.ref_x = ref_x;
        self.ref_y = ref_y;
        self.ref_z = ref_z;

class SimulationResults(object):
    """
        Simulation Results holder
    """ 
    def __init__(self,t_out_x,t_out_y,t_out_z,y_out_x,y_out_y,y_out_z,x_out_x,x_out_y,x_out_z):
        self.t_out_x = t_out_x;
        self.t_out_y = t_out_y;
        self.t_out_z = t_out_z;
        self.y_out_x = y_out_x;
        self.y_out_y = y_out_y;
        self.y_out_z = y_out_z;
        self.x_out_x = x_out_x;
        self.x_out_y = x_out_y;
        self.x_out_z = x_out_z;

class Filter2ndOrder(object):
    """
        Second order filter based on 2 time constants
    """ 
    def __init__(self,Ts1,Ts2,dc_gain,Ts):
        self.Ts1 = Ts1;
        self.Ts2 = Ts2;
        self.dc_gain = dc_gain;
        self.Ts = Ts;

    def RunFilter(self,x_vec,t_vec):
        den = signal.convolve(np.array([self.Ts1,1]),np.array([self.Ts2,1]));
        num = self.dc_gain;
        filter_obj = signal.cont2discrete((num,den),self.Ts,method='zoh');
        tout, x_filt= signal.dlsim(filter_obj,x_vec,t=t_vec);
        return x_filt;



class SystemDynamics(object):
    """
        System Dynamics
    """ 
    def __init__(self, VehicleParameters, EnvironmentParameters,TuningParameters,SimulationParameters):
        #   x-axis
        A_x = np.zeros((2,2)); A_x[0,1] = 1;
        B_x = np.zeros((2,1)); B_x[1] = -EnvironmentParameters.g; 
        C_x = np.zeros((2,2)); C_x[0,0] = 1; C_x[1,1] = 1;
        D_x = np.zeros((2,1));
        sys_x_d = signal.cont2discrete((A_x,B_x,C_x,D_x),TuningParameters.Ts,"zoh");

        #   y-axis
        A_y = np.zeros((2,2)); A_y[0,1] = 1;
        B_y = np.zeros((2,1)); B_y[1] = EnvironmentParameters.g; 
        C_y = np.zeros((2,2)); C_y[0,0] = 1; C_y[1,1] = 1;
        D_y = np.zeros((2,1));
        sys_y_d = signal.cont2discrete((A_y,B_y,C_y,D_y),TuningParameters.Ts,"zoh");

        #   z-axis
        A_z = np.zeros((2,2)); A_z[0,1] = 1;
        B_z = np.zeros((2,1)); B_z[1] = 1; 
        C_z = np.zeros((2,2)); C_z[0,0] = 1; C_z[1,1] = 1;
        D_z = np.zeros((2,1));
        sys_z_d = signal.cont2discrete((A_z,B_z,C_z,D_z),TuningParameters.Ts,"zoh");

        self.VehicleParameters = VehicleParameters;
        self.EnvironmentParameters = EnvironmentParameters;
        self.TuningParameters = TuningParameters;
        self.SimulationParameters = SimulationParameters;
        self.SysX_d = sys_x_d;
        self.SysY_d = sys_y_d;
        self.SysZ_d = sys_z_d;

    def SimulateDynamics(self):
        t_out_x, y_out_x, x_out_x = signal.dlsim(self.SysX_d, self.SimulationParameters.u_vec, t=self.SimulationParameters.t_vec);
        t_out_y, y_out_y, x_out_y = signal.dlsim(self.SysY_d, self.SimulationParameters.u_vec, t=self.SimulationParameters.t_vec);
        t_out_z, y_out_z, x_out_z = signal.dlsim(self.SysZ_d, self.SimulationParameters.u_vec, t=self.SimulationParameters.t_vec);
        self.SimOL = SimulationResults(t_out_x,t_out_y,t_out_z,y_out_x,y_out_y,y_out_z,x_out_x,x_out_y,x_out_z);

    def PlotOL(self):
        plt.subplot(321);
        plt.plot(self.SimOL.t_out_x,self.SimOL.y_out_x[:,0],'ro');
        plt.xlabel('Time (s)'); plt.ylabel('x (m)');
        plt.subplot(322);
        plt.plot(self.SimOL.t_out_x,self.SimOL.y_out_x[:,1],'ro');
        plt.xlabel('Time (s)'); plt.ylabel('v_x (m/s)');
        plt.subplot(323);
        plt.plot(self.SimOL.t_out_y,self.SimOL.y_out_y[:,0],'bo');
        plt.xlabel('Time (s)'); plt.ylabel('y (m)');
        plt.subplot(324);
        plt.plot(self.SimOL.t_out_y,self.SimOL.y_out_y[:,1],'bo');
        plt.xlabel('Time (s)'); plt.ylabel('v_y (m/s)');
        plt.subplot(325);
        plt.plot(self.SimOL.t_out_z,self.SimOL.y_out_z[:,0],'go');
        plt.xlabel('Time (s)'); plt.ylabel('z (m)');
        plt.subplot(326);
        plt.plot(self.SimOL.t_out_z,self.SimOL.y_out_z[:,1],'go');
        plt.xlabel('Time (s)'); plt.ylabel('v_z (m/s)');
        print "Close the window to continue ...";
        plt.show();
        raw_input("... and press Enter");

class LQctrl(object):
    """
        LQ Controller derivation
    """ 
    def __init__(self,sys,Q,R):
        Q1 = np.matrix(Q);
        R1 = np.matrix(R);
        A = sys[0]; B = sys[1]; 
        P = dare.DareSolver( A, B, Q1, R1).solve_direct();

        self.K = (R1 + B.transpose() * P *B).I * (B.transpose() * P * A);

    def ControlAction(self,x):
        return -np.dot(self.K,x);

def plot3(a,b,c,mark="o",col="r"):
    """
        plot3 - like MATLAB
    """ 
    # mimic matlab plot3
    from matplotlib import pyplot
    import pylab
    from mpl_toolkits.mplot3d import Axes3D
    pylab.ion();
    fig = pylab.figure();
    ax = Axes3D(fig);
    ax.plot(a, b,c,color=col,marker=mark);
    fig.show();

def plot_result(SimResults,ref_xyz,mark="o",col="r",mark_ref="*",col_ref="b"):
    """
        2x plot3
    """ 
    # mimic matlab plot3
    from matplotlib import pyplot
    import pylab
    from mpl_toolkits.mplot3d import Axes3D
    pylab.ion();
    fig = pylab.figure();
    ax = Axes3D(fig);
    ax.plot(SimResults[:,0], SimResults[:,2],SimResults[:,4],color=col,marker=mark);
    ax.plot(ref_xyz[:,0],ref_xyz[:,1],ref_xyz[:,2],color=col_ref,marker=mark_ref);
    fig.show();

class ClosedLoopSim(object):
    """
        Closed-loop Simulation
    """ 
    def __init__(self,sys,ctrl,x_init,ref_pos,Ts,accur,max_ctrl,max_err):
        self.sys = sys;
        self.ctrl = ctrl;
        self.x_init = x_init;
        self.ref_pos = ref_pos;
        self.Ts = Ts;
        self.accur = accur;
        self.max_ctrl = max_ctrl;
        self.max_err = max_err;

    def Simulate(self):
        A = self.sys[0];
        B = self.sys[1];
        C = self.sys[2];
        D = self.sys[3];
        x_init = self.x_init;
        error_vec = np.zeros((2,1))
        x_tmp = np.zeros((1000000,2));
        t_tmp = np.zeros((1000000,1));
        cnt = 2;
        x_tmp[0,0] = x_init[0]; x_tmp[0,1] = x_init[1];
        x_tmp[1,0] = x_init[0]; x_tmp[1,1] = x_init[1];
        while (abs(x_init[0] - self.ref_pos) > self.accur) & (cnt<= 100000):

            error_vec[0] = self.ref_pos - x_init[0]; error_vec[1] = 0 - x_init[1];
            error_vec[0] = max(error_vec[0],-self.max_err); error_vec[0] = min(error_vec[0],self.max_err);
            u_ctrl = self.ctrl.ControlAction(-error_vec);
            u_ctrl = min(u_ctrl,self.max_ctrl); u_ctrl = max(u_ctrl,-self.max_ctrl);
            x_tmp[cnt,1] = A[1,0]*x_init[0] + A[1,1]*x_init[1] + B[1]*u_ctrl*self.Ts;
            x_tmp[cnt,0] = A[0,0]*x_init[0] + A[0,1]*x_init[1] + B[0]*u_ctrl*self.Ts;
            x_init[0] = x_tmp[cnt,0]; x_init[1] = x_tmp[cnt,1];
            t_tmp[cnt] = cnt*self.Ts;
            cnt = cnt + 1;

        x_out = x_tmp[:(cnt-1),:];
        t_out = t_tmp[:(cnt-1)];
        return x_out, t_out;






def main(argv): 
    """
        Main method
    """

    # read the input file
    inputfile = '';
    outputfile = '';
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="]);
    except getopt.GetoptError:
        print 'test.py -i <inputfile> -o <outputfile>'
        sys.exit(2);
    for opt, arg in opts:
        if opt == '-h':
            print 'test.py -i <inputfile> -o <outputfile>'
            sys.exit();
        elif opt in ("-i", "--ifile"):
            inputfile = arg;
        elif opt in ("-o", "--ofile"):
            outputfile = arg;

    global exported_trajectory_file
    exported_trajectory_file = open(outputfile,"w");
    global ref_xyz
    ref_xyz = np.genfromtxt(inputfile, delimiter = ',');

    x_init_x[0] = ref_xyz[0,0]; x_init_y[0] = ref_xyz[0,1]; x_init_z[0] = ref_xyz[0,2]; 
    Vehicle = VehicleParameters(mass,max_xy_vel,max_z_vel,radians(maxRollPitch),maxDeltaT);
    Environment = EnvironmentParameters(9.8065);
    Tuning = TuningParameters(sample_time,Q_x,R_x,Q_y,R_y,Q_z,R_z,max_error_xyz);
    filt = Filter2ndOrder(1,2,1,Tuning.Ts);

    t_vec = np.arange(0,10,Tuning.Ts); # used only for open-loop simulations
    u_vec = np.ones((len(t_vec),1)); u_vec[0] = 0; # used only for open-loop simulations

    SimulationPars = SimulationParameters(Tuning.Ts,t_vec,u_vec,ref_xyz[:,0],ref_xyz[:,1],ref_xyz[:,2]);
    OverallSys = SystemDynamics(VehicleParameters = Vehicle, EnvironmentParameters = Environment, TuningParameters = Tuning, SimulationParameters = SimulationPars);

    Xctrl = LQctrl(OverallSys.SysX_d,Tuning.Qx,Tuning.Rx);
    Yctrl = LQctrl(OverallSys.SysY_d,Tuning.Qy,Tuning.Ry);
    Zctrl = LQctrl(OverallSys.SysZ_d,Tuning.Qz,Tuning.Rz);

    [Nr,Ny] = ref_xyz.shape;

    X_vec = np.zeros((1,2)); X_vec[0,0] = x_init_x[0]; X_vec[0,1] = x_init_x[1];
    Y_vec = np.zeros((1,2)); Y_vec[0,0] = x_init_y[0]; Y_vec[0,1] = x_init_y[1];
    Z_vec = np.zeros((1,2)); Z_vec[0,0] = x_init_z[0]; Z_vec[0,1] = x_init_z[1];
    Yaw_vec = np.zeros((1,1)); Yaw_vec[0,0] = x_init_yaw[0];
    SimResults = np.zeros((1,7)); 
    SimResults[0,0] = x_init_x[0]; SimResults[0,1] = x_init_x[1];
    SimResults[0,2] = x_init_y[0]; SimResults[0,3] = x_init_y[1];
    SimResults[0,4] = x_init_z[0]; SimResults[0,5] = x_init_z[1]; 
    SimResults[0,6] = x_init_yaw[0];
    two_indices = np.zeros((1,2)); two_yaw_refs = np.zeros((1,2));
    # simulate the whole path and extract the trajectory
    for i in range(1,Nr):
        SysX_Cl = ClosedLoopSim(OverallSys.SysX_d,Xctrl,x_init_x,ref_xyz[i,0],Tuning.Ts,accuracy_range,Vehicle.maxPitch,max_error_xyz);
        SysY_Cl = ClosedLoopSim(OverallSys.SysY_d,Yctrl,x_init_y,ref_xyz[i,1],Tuning.Ts,accuracy_range,Vehicle.maxRoll,max_error_xyz);
        SysZ_Cl = ClosedLoopSim(OverallSys.SysZ_d,Zctrl,x_init_z,ref_xyz[i,2],Tuning.Ts,accuracy_range,Vehicle.maxDeltaT,max_error_xyz);
        x_vec, t_x = SysX_Cl.Simulate(); x_init_x[0] = x_vec[len(t_x)-1,0]; x_init_x[1] = x_vec[len(t_x)-1,1]; 
        y_vec, t_y = SysY_Cl.Simulate(); x_init_y[0] = y_vec[len(t_y)-1,0]; x_init_y[1] = y_vec[len(t_y)-1,1]; 
        z_vec, t_z = SysZ_Cl.Simulate(); x_init_z[0] = z_vec[len(t_z)-1,0]; x_init_z[1] = z_vec[len(t_z)-1,1]; 
        max_len = max(len(x_vec),len(y_vec)); max_len = max(max_len,len(z_vec));
        val_vec_x = np.linspace(0,len(x_vec),len(x_vec));
        val_vec_y = np.linspace(0,len(y_vec),len(y_vec));
        val_vec_z = np.linspace(0,len(z_vec),len(z_vec));
        x_pos_interp = np.interp(np.linspace(0,len(x_vec),max_len),val_vec_x,x_vec[:,0]);
        x_vel_interp = np.interp(np.linspace(0,len(x_vec),max_len),val_vec_x,x_vec[:,1]);
        y_pos_interp = np.interp(np.linspace(0,len(y_vec),max_len),val_vec_y,y_vec[:,0]);
        y_vel_interp = np.interp(np.linspace(0,len(y_vec),max_len),val_vec_y,y_vec[:,1]);
        z_pos_interp = np.interp(np.linspace(0,len(z_vec),max_len),val_vec_z,z_vec[:,0]);
        z_vel_interp = np.interp(np.linspace(0,len(z_vec),max_len),val_vec_z,z_vec[:,1]);
        two_indices[0,0] = 0; two_indices[0,1] = max_len; two_yaw_refs[0,0] = ref_xyz[i-1,3]; two_yaw_refs[0,1] = ref_xyz[i,3];
        yaw_vec_interp = np.interp(np.linspace(0,max_len,max_len),two_indices[:,0],two_yaw_refs[:,0]);
        sim_results = np.zeros((len(x_pos_interp),7));
        sim_results[:,0] = x_pos_interp; sim_results[:,1] = x_vel_interp;
        sim_results[:,2] = y_pos_interp; sim_results[:,3] = y_vel_interp;
        sim_results[:,4] = z_pos_interp; sim_results[:,5] = z_vel_interp;
        sim_results[:,6] = yaw_vec_interp;
        SimResults = np.vstack((SimResults,sim_results));


    t_out = np.linspace(0,len(SimResults)*Tuning.Ts,len(SimResults));

    # pass all states from a second order filter
    SimResults_Filter = np.zeros((len(SimResults)+1,8));  
    SimResults_Filter[:,0] = filt.RunFilter(SimResults[:,0],t_out).transpose();
    SimResults_Filter[:,1] = filt.RunFilter(SimResults[:,1],t_out).transpose();
    SimResults_Filter[:,2] = filt.RunFilter(SimResults[:,2],t_out).transpose();
    SimResults_Filter[:,3] = filt.RunFilter(SimResults[:,3],t_out).transpose();
    SimResults_Filter[:,4] = filt.RunFilter(SimResults[:,4],t_out).transpose();
    SimResults_Filter[:,5] = filt.RunFilter(SimResults[:,5],t_out).transpose();
    SimResults_Filter[:,6] = filt.RunFilter(SimResults[:,6],t_out).transpose();
    SimResults_Filter[:,7] = np.linspace(0,len(SimResults)*Tuning.Ts,len(SimResults)+1);
    np.savetxt(exported_trajectory_file,SimResults_Filter, delimiter = ',')

    # plot the results
    plot_result(SimResults_Filter[1:len(SimResults_Filter),:],ref_xyz,'o','g','*','b');
    raw_input("Press Enter to close");




if __name__ == "__main__":
    main(sys.argv[1:])

