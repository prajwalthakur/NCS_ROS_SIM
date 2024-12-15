import numpy as np
from dataclasses import dataclass


from delay_models import constant_delay , turncated_normal_delay

class delay_model():
    def __init__(self,): 
        self.feedback_min_delay = 0.001
        self.feedback_mean_delay = 0.05
        self.feedback_max_delay = 0.070
        self.feedback_std_deviation_delay = 0.02
        
        self.input_min_delay = 0.35
        self.input_mean_delay = 0.5
        self.input_max_delay = 0.45
        self.input_std_deviation_delay = 0.001
    
        self.feedback_delay = constant_delay(delay=0.001)     #turncated_normal_delay(low=self.feedback_min_delay, mu =  self.feedback_mean_delay,  sigma=self.feedback_std_deviation_delay , high= self.feedback_max_delay)  #turncated_normal_delay(low=0.14, mu = 0.15,  sigma=0.05 , high=0.160) #turncated_normal_delay(low=0.1, mu = 0.15,  sigma=0.05 , high=0.199) #turncated_normal_delay(low=0.01, mu = 0.35,  sigma=0.05 , high=0.399) #turncated_normal_delay(low=0.01,mu = 0.15,  sigma=0.05 , high=0.199) #constant_delay(delay=0.3) #
        self.input_delay    = constant_delay(delay=0.5) #turncated_normal_delay(low= self.input_min_delay , mu = self.input_mean_delay ,  sigma= self.input_std_deviation_delay , high=self.input_max_delay )  #turncated_normal_delay(low=0.19 ,mu = 0.2,  sigma=0.05 , high=0.21)  #turncated_normal_delay(low=0.40, mu = 0.45,  sigma=0.05 , high=0.499)  #onstant_delay(delay=0.2) #



class ExperimentClass():
    
    def __init__(self):
        # Baseline-A
        
        self.do_upsampling = False
        self.baseline_exp = False
        #self.experiment_name = "baselineA" 
        self.experiment_name = "baseline0" 
        if self.experiment_name== "baseline0":
            self.mpc_mode =  "constant-input-linearization"  #"constant-input-linearization" #constant-input-linearization" #"seq-input-linearization" #"constant-input-linearization" #"constant-input-linearization" #"seq-input-linearization" #"constant-input-linearization" #zero-input-linearization 
            self.fwd_sim_mode = "constant_steering_and_speed_mode" , #zero input
            #self.fwd_sim_mode = "no_fwd_sim" # "constant_speed_and_steering": #"with_last_input_"
            self.sequence_or_one_ref = "one_ref"
            self.with_delay_compensation = False        
 
        # if self.experiment_name== "baselineA":
        #     self.mpc_mode ="constant-input-linearization" #"constant-input-linearization" #"seq-input-linearization" #"constant-input-linearization" #zero-input-linearization 
        #     self.fwd_sim_mode = "constant_steering_and_speed_mode" , #zero input
        #     self.sequence_or_one_ref = "one_ref"
        #     self.with_delay_compensation = False
        
        # if self.experiment_name=="proposed":
        #     self.mpc_mode ="constant-input-linearization" #"constant-input-linearization" #"seq-input-linearization" #"constant-input-linearization" #zero-input-linearization 
        #     self.fwd_sim_mode = "control_from_buffer" , # given a state , fwd simulate the dynamics from the control available in buffer
        #     self.sequence_or_one_ref = "seq_ref"
        
        # if self.experiment_name=="proposed_v2":
        #     self.mpc_mode ="sequence_input_linearization" #"constant-input-linearization" #"seq-input-linearization" #"constant-input-linearization" #zero-input-linearization 
        #     self.fwd_sim_mode = "control_from_buffer" , # given a state , fwd simulate the dynamics from the control available in buffer
        #     self.sequence_or_one_ref = "seq_ref"
class SimParameters():
    def __init__(self,with_pid):
        ExperimentClass.__init__(self)
        self.sim_dt = 0.01
        self.sensor_dt = 0.1
        self.delay_res_dt = 1e-4
        self.plot_update_dt = 0.05
        self.init_ego_pose =  np.array([1, 0.193,  -0.6562398 ,0.0,0.0,-0.0,0.0]) #np.array([1, 0.193,  -0.6562398 ,0.0,0.0,-0.065,0.065])
        # np.array([5, 0.193,  0.0,0.0,0.0,0,0]) #  np.array([5, 0.193,  -0.6562398 ,0.0,0.0,0,0]) #
        #self.init_closest_index = 3
        self.init_controller_control = np.array([0.0,0.0,0.0,0.0])
        self.init_control_ref_to_veh = np.array([0.0,0.0,-0.00,0.00])
        
        ############ type of experiment
        if with_pid:
                        
            self.sx = 7 # internal
            self.su = 4 # internal
        else :
            self.sx = 5
            self.su = 4


class MPCParams(SimParameters):
    def __init__(self):
        self.with_pid = True # true
        SimParameters.__init__(self,self.with_pid)
        delay_model.__init__(self)
        if self.with_pid:    
            # car physical parameter
            self.Length  = 2.4
            self.width   = 0.9
            self.lf = self.Length/2
            self.lr = self.lf
            self.lwb  = self.lf + self.lr

            # simulation type cofig

            self.with_delay_in_system  = False
            self.mpc_track_res = 0.05

            self.control_dt = 0.1
            
            # State and control dimensions


            

            self.mpc_costScale = 1
            self.terminal_cost_scale = 50
            self.NX = 5
            self.NU = 4
            
            self.ax  =5
            self.au = 4
        

            # Horizon length
            self.horizon_length = 10 
            
            # Vehicle parameters

            self.N_IND_SEARCH = 50
            
            # Initial conditions
            self.start_idx = 3

            #  SFM_weight = 90
            # sigma_yy_1 = 0.25
            # Q = np.kron(np.eye(Np), np.diag([5, 10, 30, 80, 80]))
            # R = 5 * np.kron(np.eye(Nc), np.diag([.1, .1, 5, 5]))           
            # Weighting matrices
            # self.R = 5*np.diag([.1, .1, 5, 5]) #np.diag([0.1, 0.1, 0.2, 0.2])  # Control magnitude weighting
            # self.Rd = 1 * np.diag([60, 60, 4, 4])  # Rate of change of control weighting
            # self.Q = np.diag([5, 10, 30, 80, 80]) 
            # self.Q_terminal =  self.Q  
            
            ############# mine
            self.R = 5*np.diag([.1, .1, 5, 5]) #np.diag([0.1, 0.1, 0.2, 0.2])  # Control magnitude weighting
            self.Rd = 10 * np.diag([60, 60, 4, 4])  # Rate of change of control weighting
            #self.Rd = 25* np.diag([20, 20, 15, 15])  #10* np.diag([30, 30, 40, 40])  # Rate of change of control weighting

            self.Q = np.diag([5, 10, 30, 80, 80]) #np.diag([5, 10, 30, 80, 80])#np.diag([15, 15, 50, 40,40])   # State-error cost matrix
            self.Q_terminal =  self.Q  #np.diag([15, 15, 50, 80,80])
            
            
            # State limits
            self.max_x = 1e8
            self.max_y = 1e8
            self.max_yaw = 1e8  # Can be set to np.pi/2 for more realistic bounds
            self.max_Vf = 2.5 / 3.6 #m/sec
            self.max_Vr = 2.5 / 3.6 #m/sec
            
            self.min_x = -1e8
            self.min_y = -1e8
            self.min_yaw = -np.pi/2  # Can be set to -np.pi/2 for more realistic bounds
            self.min_Vf = -0.1 #m/sec
            self.min_Vr = -0.1 #m/sec
            
            # Control limits
            self.max_Vfdot = 10.0/3.6 #m/sec^2
            self.max_Vrdot = 10.0/3.6 #m/sec^2
            self.max_steerf = np.pi / 2
            self.max_steerr = np.pi / 2
            
            self.min_Vfdot = - self.max_Vfdot
            self.min_Vrdot = - self.max_Vfdot
            self.min_steerf = - self.max_steerf
            self.min_steerr = - self.max_steerf
            
            # Control rate of change limits
            self.max_Vfddot = 20.0/3.6 #m/sec^2
            self.max_Vrddot = 20.0 / 3.6 #m/sec^2
            self.max_steerfdot = np.pi / 12
            self.max_steerrdot = np.pi / 12
            
            self.min_Vfddot = - self.max_Vfddot
            self.min_Vrddot = - self.max_Vrddot 
            self.min_steerfdot = -self.max_steerfdot 
            self.min_steerrdot = -self.max_steerrdot 
            self.ref_v = 0.8
                    
            self.lbx =  [ self.min_x, self.min_y, self.min_yaw, self.min_Vf, self.min_Vr,self.min_Vfdot, self.min_Vrdot, self.min_steerf, self.min_steerr]
            self.ubx =[self.max_x, self.max_y, self.max_yaw, self.max_Vf, self.max_Vr,self.max_Vfdot, self.max_Vrdot, self.max_steerf, self.max_steerr]
            self.Tx = np.array([1.0,1.0,1/(2*np.pi),1/self.max_Vf , 1/self.max_Vr])
            self.Tu = np.array([1/self.max_Vfdot,1/self.max_Vrdot,1/self.max_steerf,1/self.max_steerr])
            self.invTx = np.array([1.0,1.0,2*np.pi,self.max_Vf,self.max_Vr])
            self.invTu = np.array([self.max_Vfdot,self.max_Vrdot,self.max_steerf,self.max_steerr])      
            self.TDu =  np.array([1,3])
            self.invTDu = np.array([1,3])
            self.lbu = [ self.min_Vfddot, self.min_Vrddot, self.min_steerfdot, self.min_steerrdot ]
            self.ubu = [ self.max_Vfddot, self.max_Vrddot, self.max_steerfdot, self.max_steerrdot ]
            ################################### V2
            # self.lbx =  [ self.min_x, self.min_y, self.min_yaw, self.min_Vf*self.control_dt, self.min_Vr*self.control_dt,self.min_Vfdot*self.control_dt,\
            #     self.min_Vrdot*self.control_dt, self.min_steerf, self.min_steerr]
            # self.ubx =[self.max_x, self.max_y, self.max_yaw, self.max_Vf*self.control_dt, self.max_Vr*self.control_dt,self.max_Vfdot*self.control_dt, \
            #     self.max_Vrdot*self.control_dt, self.max_steerf, self.max_steerr]
            # self.Tx = np.array([1.0,1.0,1/(2*np.pi),1/(self.max_Vf*self.control_dt) , 1/(self.max_Vr*self.control_dt)])
            # self.Tu = np.array([1/(self.max_Vfdot*self.control_dt),1/(self.max_Vrdot*self.control_dt),1/self.max_steerf,1/self.max_steerr])
            # self.invTx = np.array([1.0,1.0,2*np.pi,self.max_Vf*self.control_dt,self.max_Vr*self.control_dt])
            # self.invTu = np.array([self.max_Vfdot*self.control_dt,self.max_Vrdot*self.control_dt,self.max_steerf,self.max_steerr])      
            # self.TDu =  np.array([1,3])
            # self.invTDu = np.array([1,3])
            # self.lbu = [ self.min_Vfddot*self.control_dt, self.min_Vrddot*self.control_dt, self.min_steerfdot*self.control_dt, self.min_steerrdot*self.control_dt ]
            # self.ubu = [ self.max_Vfddot*self.control_dt, self.max_Vrddot*self.control_dt, self.max_steerfdot*self.control_dt, self.max_steerrdot*self.control_dt]
            # self.lbx =  [ self.min_x, self.min_y, self.min_yaw, self.min_Vf, self.min_Vr,self.min_Vfdot,\
            #     self.min_Vrdot, self.min_steerf, self.min_steerr]
            # self.ubx =[self.max_x, self.max_y, self.max_yaw, self.max_Vf, self.max_Vr,self.max_Vfdot, \
            #     self.max_Vrdot, self.max_steerf, self.max_steerr]
            # self.invTx = np.array([1.0,1.0,2*np.pi,self.max_Vf,self.max_Vr])
            # self.Tx = np.array([1.0,1.0,1/(2*np.pi),1/(self.max_Vf) , 1/(self.max_Vr)])
            
            # self.Tu = np.array([1/(self.max_Vfdot),1/(self.max_Vrdot),1/self.max_steerf,1/self.max_steerr])
            # self.invTu = np.array([self.max_Vfdot,self.max_Vrdot,self.max_steerf,self.max_steerr])      
            
            # # actual control since I augment and work on the 
            # self.TDu =  np.array([1,3])
            # self.invTDu = np.array([1,3])
            # self.lbu = [ self.min_Vfddot*self.control_dt, self.min_Vrddot*self.control_dt, self.min_steerfdot*self.control_dt, self.min_steerrdot*self.control_dt ]
            # self.ubu = [ self.max_Vfddot*self.control_dt, self.max_Vrddot*self.control_dt, self.max_steerfdot*self.control_dt, self.max_steerrdot*self.control_dt]
        else:


            # car physical parameter
            self.Length  = 2.4
            self.width   = 0.9
            self.lf = self.Length/2
            self.lr = self.lf
            self.lwb  = self.lf + self.lr

            # simulation type cofig

            self.with_delay_in_system  = False
            self.mpc_track_res = 0.1
            self.sim_dt = 0.01
            self.control_dt = 0.1
            
            # State and control dimensions
            

            

            self.mpc_mode = "constant-input-linearization" #"constant-input-linearization" #zero-input-linearization #seq-input-linearization
            self.mpc_costScale = 1
            self.terminal_cost_scale =1
            self.NX = 5
            self.NU = 4
            
            self.ax  =5
            self.au = 4
        
            self.sim_dt = 0.01
            # Horizon length
            self.horizon_length =50
            
            # Vehicle parameters

            self.N_IND_SEARCH = 50
            
            # Initial conditions


            
            # Weighting matrices
            self.R = np.diag([0.1, 0.1, 0.2, 0.2])  # Control magnitude weighting
            self.Rd = 10* np.diag([60, 60, 10, 10])  # Rate of change of control weighting
            #self.Rd = 1* np.diag([60, 60, 1, 1])  #10* np.diag([30, 30, 40, 40])  # Rate of change of control weighting
            
            self.Q = np.diag([10, 10, 30, 20,20])   # State-error cost matrix
            self.Q_terminal =  self.Q  #np.diag([15, 15, 50, 80,80])
            # State limits
            self.max_x = 1e8
            self.max_y = 1e8
            self.max_yaw = 1e8  # Can be set to np.pi/2 for more realistic bounds
            self.max_Vf = 1.5
            self.max_Vr = 1.5
            
            self.min_x = -1e8
            self.min_y = -1e8
            self.min_yaw = -1e8  # Can be set to -np.pi/2 for more realistic bounds
            self.min_Vf = -0.1
            self.min_Vr = -0.1
            
            # Control limits
            self.max_Vfdot = 0.7
            self.max_Vrdot = 0.7
            self.max_steerf = np.pi / 2
            self.max_steerr = np.pi / 2
            
            self.min_Vfdot = - self.max_Vfdot
            self.min_Vrdot = - self.max_Vfdot
            self.min_steerf = -np.pi / 2
            self.min_steerr = -np.pi / 2
            
            # Control rate of change limits
            self.max_Vfddot = 0.8
            self.max_Vrddot = 0.8
            self.max_steerfdot = np.pi / 12
            self.max_steerrdot = np.pi / 12
            
            self.min_Vfddot = -0.8
            self.min_Vrddot = -0.8
            self.min_steerfdot = -np.pi / 12
            self.min_steerrdot = -np.pi / 12
            self.ref_v = 0.6
                    
            # self.lbx =  [ self.min_x, self.min_y, self.min_yaw, self.min_Vf, self.min_Vr,self.min_Vfdot, self.min_Vrdot, self.min_steerf, self.min_steerr]
            # self.ubx =[self.max_x, self.max_y, self.max_yaw, self.max_Vf, self.max_Vr,self.max_Vfdot, self.max_Vrdot, self.max_steerf, self.max_steerr]

            

            # self.Tx = np.array([1.0,1.0,1/(2*np.pi),1/self.max_Vf , 1/self.max_Vr])
            # self.Tu = np.array([1/self.max_Vfdot,1/self.max_Vrdot,1/self.max_steerf,1/self.max_steerr])
            # self.invTx = np.array([1.0,1.0,2*np.pi,self.max_Vf,self.max_Vr])
            # self.invTu = np.array([self.max_Vfdot,self.max_Vrdot,self.max_steerf,self.max_steerr])
            
            # self.lbx =  [-1e3, -1e3, -1, -1, -1,-1, -1, -1, -1 ]
            # self.ubx =[1e3, 1e3, 1 , 1, 1 , 1, 1 , 1 , 1 ]
            
            
            # self.TDu =  np.array([1,3])
            # self.invTDu = np.array([1,3])
            
            # self.lbu = [ self.min_Vfddot, self.min_Vrddot, self.min_steerfdot, self.min_steerrdot ]
            
            # self.ubu = [ self.max_Vfddot, self.max_Vrddot, self.max_steerfdot, self.max_steerrdot ]
            # self.lbx =  [ self.min_x, self.min_y, self.min_yaw, self.min_Vf, self.min_Vr,self.min_Vfdot, self.min_Vrdot, self.min_steerf, self.min_steerr]
            # self.ubx =[self.max_x, self.max_y, self.max_yaw, self.max_Vf, self.max_Vr,self.max_Vfdot, self.max_Vrdot, self.max_steerf, self.max_steerr]
            # self.Tx = np.array([1.0,1.0,1/(2*np.pi),1/self.max_Vf , 1/self.max_Vr])
            # self.Tu = np.array([1/self.max_Vfdot,1/self.max_Vrdot,1/self.max_steerf,1/self.max_steerr])
            # self.invTx = np.array([1.0,1.0,2*np.pi,self.max_Vf,self.max_Vr])
            # self.invTu = np.array([self.max_Vfdot,self.max_Vrdot,self.max_steerf,self.max_steerr])      
            # self.TDu =  np.array([1,3])
            # self.invTDu = np.array([1,3])
            # self.lbu = [ self.min_Vfddot, self.min_Vrddot, self.min_steerfdot, self.min_steerrdot ]
            # self.ubu = [ self.max_Vfddot, self.max_Vrddot, self.max_steerfdot, self.max_steerrdot ]
            self.lbx =  [ self.min_x, self.min_y, self.min_yaw, self.min_Vf*self.control_dt, self.min_Vr*self.control_dt,self.min_Vfdot*self.control_dt,\
                self.min_Vrdot*self.control_dt, self.min_steerf, self.min_steerr]
            self.ubx =[self.max_x, self.max_y, self.max_yaw, self.max_Vf*self.control_dt, self.max_Vr*self.control_dt,self.max_Vfdot*self.control_dt, \
                self.max_Vrdot*self.control_dt, self.max_steerf, self.max_steerr]
            self.Tx = np.array([1.0,1.0,1/(2*np.pi),1/(self.max_Vf*self.control_dt) , 1/(self.max_Vr*self.control_dt)])
            self.Tu = np.array([1/(self.max_Vfdot*self.control_dt),1/(self.max_Vrdot*self.control_dt),1/self.max_steerf,1/self.max_steerr])
            self.invTx = np.array([1.0,1.0,2*np.pi,self.max_Vf*self.control_dt,self.max_Vr*self.control_dt])
            self.invTu = np.array([self.max_Vfdot*self.control_dt,self.max_Vrdot*self.control_dt,self.max_steerf,self.max_steerr])      
            self.TDu =  np.array([1,3])
            self.invTDu = np.array([1,3])
            self.lbu = [ self.min_Vfddot*self.control_dt, self.min_Vrddot*self.control_dt, self.min_steerfdot*self.control_dt, self.min_steerrdot*self.control_dt ]
            self.ubu = [ self.max_Vfddot*self.control_dt, self.max_Vrddot*self.control_dt, self.max_steerfdot*self.control_dt, self.max_steerrdot*self.control_dt]