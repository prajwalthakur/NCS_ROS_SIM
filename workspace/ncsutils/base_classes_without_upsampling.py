from dataclasses import dataclass , field
from typing import Optional
from abc import ABC, abstractmethod
import numpy as np

from params import MPCParams
import sys









def fwd_sim_obs(x_obs,y_obs,psi_obs,vx_obs,vy_obs, delay_time):
    x_obs = x_obs + vx_obs*delay_time
    y_obs = y_obs + vy_obs*delay_time
    psi_obs_traj = psi_obs    
    x_obs_pred = x_obs
    y_obs_pred = y_obs
    psi_obs_pred = psi_obs_traj

    return x_obs_pred,y_obs_pred,psi_obs_pred,vx_obs,vy_obs


def synthesise_control_seq_by_upsampling(mpc_control_seq,controller_time_step,sim_time_sampling):

    num_controls, len_control = mpc_control_seq.shape[0], mpc_control_seq.shape[1]
    ##print(num_controls,"--",len_control)
    #pdb.set_trace()
    original_time = np.arange(0, len_control * controller_time_step, controller_time_step)  # Time for original sequence
    upsampled_time = np.arange(0, original_time[-1] + sim_time_sampling , sim_time_sampling)  # Time for upsampled sequence
    #pdb.set_trace()
    upsampled_ctrl_sequence = np.zeros((num_controls, len(upsampled_time)))
    for i in range(num_controls):
        upsampled_ctrl_sequence[i,:] = np.interp(upsampled_time, original_time, mpc_control_seq[i,:])
    return upsampled_ctrl_sequence







class ColorText:
    def __init__(self):
        # Define ANSI escape codes for styles and colors
        self.bold_text = "\033[1m"
        self.green_color = "\033[32m"
        self.reset_text = "\033[0m"  # Reset text formatting
        self.reset_color = "\033[0m" # Reset color formatting
        self.red_color = "\033[31m"    # Red color



# Define the state for an Ackerman robot
@dataclass
class RobotStateAckerman:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    steer_front: float = 0.0
    steer_rear: float = 0.0
    speed: float = 0.0
    progress: float = 0.0
    time_sampled: Optional[float] = None

# Define the state for a hospital bed robot
@dataclass
class RobotStateHospitalBed:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v_x :  float = 0.0
    v_y : float = 0.0
    v_front: float = 0.0
    v_rear: float = 0.0
    steer_front: float = 0.0
    steer_rear: float = 0.0
    # speed: float = 0.0
    # progress: float = 0.0
    time_sampled: Optional[float] = None

@dataclass
class RobotControlHospitalBed():
    _seq_length: int = field(init=True, repr=True)
    _sampling_time : np.float32 =  field(init=True, repr=True)
    a_f : np.ndarray = field(default_factory=lambda:np.zeros((1,)))
    a_r : np.ndarray = field(default_factory=lambda:np.zeros((1,)))
    s_f : np.ndarray = field(default_factory=lambda:np.zeros((1,)))
    s_r : np.ndarray = field(default_factory=lambda:np.zeros((1,)))
    time_seqs : np.ndarray = field(default_factory=lambda:np.zeros((1,)))
    msg_id : int = 0
    conrol_index_in_seqs : int = 0
    
    def __post_init__(self):
        self.a_f = np.zeros((self._seq_length,))
        self.a_r = np.zeros((self._seq_length,))
        self.s_f = np.zeros((self._seq_length,))
        self.s_r = np.zeros((self._seq_length,))
    def _update_time_seqs(self,time_seq_start):
        self.time_seqs = np.array([time_seq_start + itr*self._sampling_time for itr in range(self._seq_length)])
    @property
    def seq_length(self):
        return self._seq_length

    @seq_length.setter
    def seq_length(self, value):
        raise AttributeError("seq_length is read-only and cannot be modified.")

# Abstract robot class
class RobotObj(ABC):
    def __init__(self, vehicle_type: str,current_time:np.float32,control_seq_length:int,control_update_rate:np.float32):
        ColorText.__init__(self)
        MPCParams.__init__(self)
        self.vehicle_type = vehicle_type
        self.robot_state = self._create_robot_state()
        self.robot_control_obj = self._create_control_state_seqs(current_time, control_seq_length, control_update_rate)
    @abstractmethod
    def _create_robot_state(self):
        """Abstract method to create robot state."""
        pass
    
    @abstractmethod
    def _update_state(self):
        
        pass
    
    @abstractmethod
    def _update_control_seqs(self,current_time, control_seq_length, control_update_rate):
        """
        Updates control sequences based on the current time and message ID.
        """
        pass
    
    
    @abstractmethod
    def _create_control_state_seqs(self):
        
        pass
    
    @abstractmethod
    def _append_control_seqs(self):
        
        pass

# Ackerman robot class
class RobotAckerman(RobotObj):
    def _create_robot_state(self):
        return RobotStateAckerman()

# Hospital bed robot class
class RobotHospitalBed(RobotObj):
    def __init__(self, vehicle_type: str,current_time:np.float32,control_seq_length:int,control_update_rate:np.float32):
        super().__init__(vehicle_type,current_time=current_time,control_seq_length=control_seq_length,control_update_rate=control_update_rate)

    def _create_robot_state(self):
        return RobotStateHospitalBed()

    def _create_control_state_seqs(self,current_time,control_seq_length,control_update_rate):
        temp = RobotControlHospitalBed(_seq_length=control_seq_length,_sampling_time=1/control_update_rate)
        temp._update_time_seqs(time_seq_start=current_time)
        return temp
    
    def _update_state(self, x: float, y: float, yaw: float, 
                      v_x: float, v_y: float, 
                      v_front: float, v_rear: float, 
                      steer_front: float, steer_rear: float, 
                      msg_id : float, 
                      control_idx_in_seq : int ,
                      time_sampled: float
                      ):
        yaw = np.arctan2(np.sin(yaw),np.cos(yaw))
        self.robot_state.x = x
        self.robot_state.y = y
        self.robot_state.yaw = yaw
        self.robot_state.v_x = v_x
        self.robot_state.v_y = v_y
        self.robot_state.v_front = v_front
        self.robot_state.v_rear = v_rear
        self.robot_state.steer_front = steer_front
        self.robot_state.steer_rear = steer_rear
        self.robot_state.time_sampled = time_sampled
        #self._update_control_seqs(self.robot_state.time_sampled,msg_id=msg_id,control_itr=control_idx_in_seq)
        self._update_control_seqs(self.robot_state.time_sampled,msg_id=None,control_itr=None)

    
    def _create_control_seqs(self,control_seqs,time_seq_start ):
        a_f = control_seqs[0,:]
        a_r = control_seqs[1,:]
        s_f = control_seqs[2,:]
        s_r = control_seqs[3,:]
        time_seqs = np.array([time_seq_start + itr*self.robot_control_obj._sampling_time for itr in range(control_seqs.shape[1])])
        #print(f'{self.bold_text}{self.red_color}_create_control_seqs{control_seqs.shape}.helllloo{control_seqs.shape[0]},time-seqs.shape{time_seqs}{self.reset_color}{self.reset_text}')
        return a_f,a_r,s_f,s_r,time_seqs
    def _update_control_seqs(self,time_t:np.float32,msg_id:np.float32,control_itr:int)->None:
        if msg_id==None and control_itr==None:
            temp =self.robot_control_obj.time_seqs - time_t
            print(f'{self.bold_text}{self.red_color}_update_control_seqs{temp}{self.reset_color}{self.reset_text}')
            print(f'{self.bold_text}{self.red_color}self.robot_control_obj.time_seqs{self.robot_control_obj.time_seqs[0]},feedback_time {time_t}{self.reset_color}{self.reset_text}')
            
            #rospy.signal_shutdown("MPC operation completed.")
            
            positive_temp_indices = np.where(temp >= 0)[0]
            if positive_temp_indices.size > 0:
                closest_positive_index = positive_temp_indices[0] #positive_temp_indices[np.argmin(temp[positive_temp_indices])]
            else:
                closest_positive_index = -1  # Handle case where no positive values exist  
                #print(f'{self.bold_text}{self.red_color}_update_control_seqs function{self.reset_color}{self.reset_text}')
            print(f'{self.bold_text}{self.red_color}closest-positive-index{closest_positive_index}{self.reset_color}{self.reset_text}')
                    
            self.robot_control_obj.a_f = self.robot_control_obj.a_f[closest_positive_index:]
            self.robot_control_obj.a_r = self.robot_control_obj.a_r[closest_positive_index:]
            self.robot_control_obj.s_f = self.robot_control_obj.s_f[closest_positive_index:]
            self.robot_control_obj.s_r = self.robot_control_obj.s_r[closest_positive_index:]
            self.robot_control_obj.time_seqs = self.robot_control_obj.time_seqs[closest_positive_index:]

        # elif msg_id != None and control_itr != None :
        #     assert msg_id==self.robot_control_obj.msg_id
        #     self.robot_control_obj.a_f = self.robot_control_obj.a_f[control_itr:]
        #     self.robot_control_obj.a_r = self.robot_control_obj.a_r[control_itr:]
        #     self.robot_control_obj.s_f = self.robot_control_obj.s_f[control_itr:]
        #     self.robot_control_obj.s_r = self.robot_control_obj.s_r[control_itr:]
        #     self.robot_control_obj.time_seqs = self.robot_control_obj.time_seqs[control_itr:]
        # else :
        #     assert msg_id == None and control_itr == None

    def _future_update_control_seqs(self,time_t:np.float32,msg_id:np.float32,control_itr:int)->None:
        assert  (msg_id==None and control_itr==None)
        print(f'{self.bold_text}{self.red_color}in future_update_control_seqs,future_time {time_t}{self.reset_color}{self.reset_text}')
        if msg_id==None and control_itr==None:
            temp =  time_t  - self.robot_control_obj.time_seqs 
            print(f'{self.bold_text}{self.red_color}temp,{temp}{self.reset_color}{self.reset_text}')
            
            positive_temp_indices = np.where(temp >= 0)[0]
            if positive_temp_indices.size > 0:
                closest_positive_index = positive_temp_indices[-1]  #positive_temp_indices[np.argmax(temp[positive_temp_indices])]
                ##print(f'{self.bold_text}{self.red_color}positive_temp_indices,{positive_temp_indices},closest_positive_index->{closest_positive_index}{self.reset_color}{self.reset_text}')
                
            else:
                closest_positive_index = 0 #Handle case where no positive values exist 
                #print(f'{self.bold_text}{self.red_color}_future_update_control_seqs function{self.reset_color}{self.reset_text}')
            print(f'{self.bold_text}{self.red_color}closest_positive_index,{closest_positive_index},--- {self.robot_control_obj.time_seqs-time_t}{self.reset_color}{self.reset_text}')      
            self.robot_control_obj.a_f = self.robot_control_obj.a_f[:closest_positive_index+1]
            self.robot_control_obj.a_r = self.robot_control_obj.a_r[:closest_positive_index+1]
            self.robot_control_obj.s_f = self.robot_control_obj.s_f[:closest_positive_index+1]
            self.robot_control_obj.s_r = self.robot_control_obj.s_r[:closest_positive_index+1]
            self.robot_control_obj.time_seqs = self.robot_control_obj.time_seqs[:closest_positive_index+1]

    def _append_control_seqs(self,control_seqs:np.ndarray, future_start_time )->None:
        #future_start_time = curr_time + assumed_delay
        assert control_seqs.shape[0]==4
        a_f,a_r,s_f,s_r,time_seqs = self._create_control_seqs(control_seqs=control_seqs,time_seq_start= future_start_time)
        self._future_update_control_seqs(time_t=future_start_time,msg_id=None,control_itr=None)
        self.robot_control_obj.a_f = a_f #np.hstack((self.robot_control_obj.a_f,a_f))
        self.robot_control_obj.a_r = a_r #np.hstack((self.robot_control_obj.a_r,a_r))
        self.robot_control_obj.s_f = s_f #np.hstack((self.robot_control_obj.s_f,s_f))
        self.robot_control_obj.s_r = s_r #np.hstack((self.robot_control_obj.s_r,s_r))
        self.robot_control_obj.time_seqs = time_seqs #np.hstack((self.robot_control_obj.time_seqs,time_seqs))
    
    def _getXdot(self,uk,x0):
        lf = self.lf
        lr = self.lr
        #x,y,yaw,v_f,v_r
        yaw = x0[2][0]
        
        v_f = x0[3][0]
        v_r = x0[4][0]
        
        v_f_dot = uk[0][0]
        v_r_dot = uk[1][0]
        
        steer_f = uk[2][0]
        steer_r = uk[3][0]

        
        beta = np.arctan2(lr*np.tan(steer_f) + lf*np.tan(steer_r) , lf+lr) #slip angle
        
        xdot = (np.cos(yaw + beta )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / ( 2*np.cos(beta))
        ydot = (np.sin(yaw+beta)*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / (2*np.cos(beta))
        yawdot = ( np.tan(steer_f) - np.tan(steer_r) )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))/(2*(lf+lr))
        
        
        Xdot = np.array([
            [xdot],
            [ydot],
            [yawdot],
            [v_f_dot],
            [v_r_dot]
            ])
        
        return Xdot   
    def _pad_to_horizon_length(self,arr, start_index, horizon_length):
        # Extract the slice from the array
        sliced_array = arr[start_index:start_index + horizon_length]
        # Calculate the number of values to pad if needed
        pad_length = max(0, horizon_length - len(sliced_array))
        # Pad with the last value in the slice if necessary
        if pad_length > 0 and len(sliced_array) > 0:
            sliced_array = np.concatenate([sliced_array, np.full(pad_length, sliced_array[-1])])
        elif pad_length > 0:  # Handle edge case where the array is empty
            sliced_array = np.zeros(horizon_length)
        return sliced_array    
       
    def get_control_seqs_array_at_future(self, control_until_time_t ):
        
        temp = control_until_time_t - self.robot_control_obj.time_seqs 
        print(f'{self.bold_text}{self.green_color}self.robot_control_obj.time_seqs-shape{ self.robot_control_obj.time_seqs}--print temp-{temp}{self.reset_color}{self.reset_text}')
        #temp = self.robot_control_obj.time_seqs[0] + 0.4 -  self.robot_control_obj.time_seqs 
        positive_temp_indices = np.where(temp >= 0)[0]
        if positive_temp_indices.size > 0:
            closest_positive_index = positive_temp_indices[-1] #positive_temp_indices[np.argmin(temp[positive_temp_indices])]
            #print(f'{self.bold_text}{self.green_color}{ positive_temp_indices }---{closest_positive_index}{self.reset_color}{self.reset_text}')
            #print(f'{self.bold_text}{self.green_color}{temp[positive_temp_indices]}{self.reset_color}{self.reset_text}')
            
        else:
            closest_positive_index = -1  # Handle case where no positive values exist      
            #print(f'{self.bold_text}{self.red_color}check the get_control_seqs_array function {self.reset_color}{self.reset_text}')
        print(f'{self.bold_text}{self.green_color}get_control_seqs_array {temp[0:closest_positive_index+1]},closest_positive_index={closest_positive_index}{self.reset_color}{self.reset_text}')
                    
        #len_control = self.robot_control_obj.a_f[:closest_positive_index]
        control_array = np.vstack(
                (self.robot_control_obj.a_f[:closest_positive_index+1][np.newaxis,],
                self.robot_control_obj.a_r[:closest_positive_index+1][np.newaxis,],
                self.robot_control_obj.s_f[:closest_positive_index+1][np.newaxis,],
                self.robot_control_obj.s_r[:closest_positive_index+1][np.newaxis,],
                )
            )

        # tt = self.robot_control_obj.a_f[:closest_positive_index][np.newaxis,]
        # #print(f'{self.bold_text}{self.green_color}{tt.shape}{self.reset_color}{self.reset_text}')
        
        expected_control_seqs = np.vstack((
            self._pad_to_horizon_length(self.robot_control_obj.a_f, closest_positive_index+1, self.horizon_length)[np.newaxis, :],
            self._pad_to_horizon_length(self.robot_control_obj.a_r, closest_positive_index+1, self.horizon_length)[np.newaxis, :],
            self._pad_to_horizon_length(self.robot_control_obj.s_f, closest_positive_index+1, self.horizon_length)[np.newaxis, :],
            self._pad_to_horizon_length(self.robot_control_obj.s_r, closest_positive_index+1, self.horizon_length)[np.newaxis, :]
        ))
        return expected_control_seqs,control_array

       
    def get_control_seqs_array(self ):

        control_array = np.vstack(
                (self.robot_control_obj.a_f[np.newaxis,...],
                self.robot_control_obj.a_r[np.newaxis,...],
                self.robot_control_obj.s_f[np.newaxis,...],
                self.robot_control_obj.s_r[np.newaxis,...],
                )
            )
        return control_array


    
    def get_state_array(self):
        state_ = np.array([[self.robot_state.x],
                           [self.robot_state.y],
                           [self.robot_state.yaw],
                           [self.robot_state.v_front],
                           [self.robot_state.v_rear]
                           ])
        return state_
    

    #u_expected_seqs ,  state_predicted   = robot_state_obj.do_forward_sim(feedback_time = feedback_time , current_time = current_time ,predicted_delay=assumed_input_delay,do_upsampling=False)
    def do_forward_sim(self,feedback_time,current_time, predicted_input_delay,do_upsampling=False):        
        delay_until_current_time = current_time - feedback_time
        predicted_total_delay = delay_until_current_time + predicted_input_delay
        predicted_future_time = current_time + predicted_input_delay
        # print(f'{self.bold_text}{self.green_color}"delay_until_current_time" , {delay_until_current_time},predicted_total_delay {predicted_total_delay} {self.reset_color}{self.reset_text}')
        # print("insrdidddsdjnsdlkjsdlglshdglkhls")
        #u_expected_seqs,state_array = self.get_control_seqs_array(),self.get_state_array()
        #u_0 = u_expected_seqs[:,0]
        # u_0 = np.zeros((self.au,))
        # for i in range(int(predicted_total_delay/self.control_dt)):
            
        #     state_array+=self._getXdot(u_0[...,np.newaxis] , state_array)*self.control_dt
        
        u_expected_seqs,control_array = self.get_control_seqs_array_at_future(predicted_future_time) #
        state_array  = self.get_state_array()
        for i in range(control_array.shape[1]):
            state_array+=self._getXdot(control_array[:,i][...,np.newaxis] , state_array)*self.control_dt
        # print(f'{self.bold_text}{self.green_color}{int(predicted_total_delay/0.1)}--control-expected-seqs-shape{u_expected_seqs__.shape} , control_array shape {control_array__.shape}{self.reset_color}{self.reset_text}')
                        
        # if self.fwd_sim_mode=="no_fwd_sim":
        #     u_expected_seqs,state_array = self.get_control_seqs_array(),self.get_state_array()
        
        # elif self.fwd_sim_mode == "constant_steering_and_speed_mode":
        #     print("insrdidddsdjnsdlkjsdlglshdglkhls")
        #     u_expected_seqs,state_array = self.get_control_seqs_array(),self.get_state_array()
        #     u_0 = np.zeros((self.au,)) #u_expected_seqs[:,0]
            
        #     for i in range(int(predicted_total_delay/0.1)):
                
        #         state_array+=self._getXdot(u_0[...,np.newaxis] , state_array)*0.1
        #     u_expected_seqs,control_array = self.get_control_seqs_array_at_future(predicted_future_time) #
        #     print(f'{self.bold_text}{self.green_color}control-expected-seqs-shape{u_expected_seqs.shape} , control_array shape {control_array.shape}{self.reset_color}{self.reset_text}')
                    
        # elif self.fwd_sim_mode == "with_last_input_":
        #     u_expected_seqs,state_array = self.get_control_seqs_array(),self.get_state_array()
        #     u_0 = u_expected_seqs[:,0]
            
        #     for i in range(int(0.05/predicted_total_delay)):
                
        #         state_array+=self._getXdot(u_0[...,np.newaxis] , state_array)*0.05
                
        
        # elif self.fwd_sim_mode == "with_last_input_seq_sync":
        #     u_expected_seqs,control_array = self.get_control_seqs_array_at_future(predicted_future_time) #
        #     print(f'{self.bold_text}{self.green_color}control-expected-seqs-shape{u_expected_seqs.shape} , control_array shape {control_array.shape}{self.reset_color}{self.reset_text}')
            
        #     state_array  = self.get_state_array()
        #         #sim_time = 0.0
        #     itr = 0
        #     fwd_time_step = 0.0
            
        #     if do_upsampling:
        #         #pdb.set_trace()
        #         control_array = synthesise_control_seq_by_upsampling(control_array,self.control_dt,self.sim_dt)    
        #         fwd_time_step = self.sim_dt
        #     else:
        #         fwd_time_step = self.control_dt

            
        #     for i in range(control_array.shape[1]):
        #         ##print(f'{self.bold_text}{self.green_color}state-array-shape{state_array.shape}{self.reset_color}{self.reset_text}')
                
        #         state_array+=self._getXdot(control_array[:,i][...,np.newaxis] , state_array)*fwd_time_step
            
   
        return u_expected_seqs,state_array



        