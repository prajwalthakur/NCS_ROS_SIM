# this is the true model of the car
#!/usr/bin/env python3  
import numpy as np
from params import MPCParams
import copy
import scipy
from dataclasses import dataclass
import pdb
@dataclass
class robotstate:    
    x:np.float32
    y:np.float32
    yaw:np.float32
    vf : np.float32
    vr : np.float32
    steer_f  : np.float32
    steer_r : np.float32
    vx : np.float32
    vy : np.float32
    yaw_rate : np.float32

class TemporalState:
    def __init__(self, X0):
        self.x = X0[0,0]
        self.y = X0[1,0]
        self.yaw = X0[2,0]
        self.v_f  =   X0[3,0]
        self.v_r  =   X0[4,0]
        self.steer_f  =   X0[5,0]
        self.steer_r  =   X0[6,0]
        self.members = ['x', 'y','yaw','v_f','v_r','steer_f','steer_r', 'progress']
    def get_states(self,lf,lr):
        vf = self.v_f
        vr = self.v_r
        beta = np.arctan2(lr*np.tan(self.steer_f) + lf*np.tan(self.steer_r) , lf+lr) 
        vc = (vf*np.cos(self.steer_f) + vr*np.cos(self.steer_r))/(2*np.cos(beta))
        vx = vc*np.cos(self.yaw + beta )
        vy = vc*np.sin(self.yaw + beta )
        yaw_rate = vc*(np.cos(beta))*(np.tan(self.steer_f) - np.tan(self.steer_r))/(lf  + lr )
        state = robotstate(x=self.x,y=self.y,yaw=self.yaw,vf =self.v_f,vr=self.v_r,steer_f=self.steer_f,steer_r=self.steer_r,vx=vx,vy=vy,yaw_rate=yaw_rate)
        return state
@dataclass
class car_limit:
    car_min_vf_dot = -20/3.6 #m/s
    car_min_vr_dot = -20/3.6
    
    car_max_vf_dot = 20/3.6
    car_max_vr_dot = 20/3.6
    
    car_max_vf  = 10/3.6
    car_max_vr = 10/3.6
    
    car_min_vf = -10/3.6
    car_min_vr = -10/3.6

    car_min_steering_f_dot = -np.pi/12
    car_min_steering_r_dot = -np.pi/12
    
    car_max_steering_f_dot  = - car_min_steering_f_dot
    car_max_steering_r_dot  = - car_min_steering_r_dot
    
    car_min_steering_f = -np.pi/2
    car_min_steering_r = -np.pi/2
    
    car_max_steering_f = np.pi/2
    car_max_steering_r = np.pi/2

# control vfdot,vrdot,steer_f_dot,steer_r_dot
class car_model(MPCParams,TemporalState,car_limit):
    def __init__(self,initial_states:np.ndarray) :
        MPCParams.__init__(self)
        car_limit.__init__(self)
        self.car_state  = TemporalState(initial_states)
        self.min_control_array = np.array([self.car_min_vf_dot, self.car_min_vr_dot ,self.car_min_steering_f_dot,self.car_min_steering_r_dot])
        self.max_control_array = -copy.deepcopy(self.min_control_array)
        self.min_car_state = np.array([-np.inf,-np.inf,-np.inf,self.car_min_vf, self.car_min_vr , self.car_min_steering_f , self.car_min_steering_r ])
        self.max_car_state = - copy.deepcopy(  self.min_car_state ) 
        self.to_pid = np.zeros((self.su,1))    
        self.last_error = np.zeros((4,1))
        self.last_pid_call = 0.0
        self.boundaryTrackConstraint_left = np.zeros((1,3))
        self.boundaryTrackConstraint_right = np.zeros((1,3))
        self.opt_predictionPoints = np.zeros((1,3))
        self.mpcPredictionPoints = np.zeros((1,3))
        self.timeElapsed = 0
        self.mpcCostCOmponents = 0
        
        self.updatePredictionPoints()
    def updatePredictionPoints(self):
        #pdb.set_trace()
        self.predictionPoints = np.vstack((self.mpcPredictionPoints,self.boundaryTrackConstraint_left,self.boundaryTrackConstraint_right))    
        self.opt_predictionPoints = self.mpcPredictionPoints

    def updatePredictionPoints_external_call(self,predictionpts,optimal_traj_points):
        #pdb.set_trace()
        self.predictionPoints = predictionpts        
        self.opt_predictionPoints = optimal_traj_points
    def xdot(self,t,x,u):
        vf_dot = u[0,0]
        vr_dot = u[1][0]
        steer_f_dot = u[2][0]
        steer_r_dot = u[3][0]
        
        x_ = x[0]
        y_ = x[1]
        yaw = x[2]
        v_f = x[3]
        v_r = x[4]
        steer_f = x[5]
        steer_r = x[6]
        
        beta = np.arctan2(self.lr*np.tan(steer_f) + self.lf*np.tan(steer_r) , self.lf+self.lr) #slip angle
        
        xdot = (np.cos(yaw + beta )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / ( 2*np.cos(beta))
        ydot = (np.sin(yaw+beta)*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / (2*np.cos(beta))
        yawdot = ( np.tan(steer_f) - np.tan(steer_r) )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))/(2*np.cos(beta)*(self.lf+self.lr))
        
        Xdot = np.array([
            [xdot],
            [ydot],
            [yawdot],
            [vf_dot],
            [vr_dot],
            [steer_f_dot],
            [steer_r_dot]
            ])    
        return Xdot.squeeze()
    
    def clip_control(self,control):
        control = np.clip(control, self.min_control_array , self.max_control_array)
        return control
    def state_clip(self,state):
        state  = np.clip(state,self.min_car_state,self.max_car_state)
        return state
    
    def pid_control(self,control , current_time):            #print("pid call at :", current_time)
        vf_error =control[0][0]  - self.car_state.v_f 
        vr_error = control[1][0] - self.car_state.v_r  
        steer_f_error = control[2][0] - self.car_state.steer_f 
        steer_r_error = control[3][0] - self.car_state.steer_r
        
        v_f_dot , steer_f_dot = self.pid(speed= control[0][0] , steer = control[2][0] ,current_speed=self.car_state.v_f, current_steer=self.car_state.steer_f \
            , max_sv = self.car_max_steering_f_dot, max_a = self.car_max_vf_dot, max_v = self.car_max_vf, min_v=self.car_min_vf)
        
        v_r_dot , steer_r_dot = self.pid(speed= control[1][0] , steer = control[3][0] ,current_speed=self.car_state.v_r, current_steer=self.car_state.steer_r \
            , max_sv = self.car_max_steering_r_dot, max_a = self.car_max_vr_dot, max_v = self.car_max_vr, min_v=self.car_min_vr)
            
        
        error = np.array([vf_error, vr_error, steer_f_error, steer_r_error])[...,np.newaxis]
        #p_term = np.array([10*vf_error, 10*vr_error, 100*steer_f_error, 100*steer_r_error])[...,np.newaxis] #np.round(self.Kp * error,4)
        #d_term = self.Kd * (error - self.last_error) *(1/ self.pid_ctr) 
        #self.to_pid = p_term + d_term
        #pdb.set_trace()
        self.to_pid = np.array([v_f_dot,v_r_dot,steer_f_dot,steer_r_dot])
        #print("before cliiping", self.to_pid)
        self.to_pid = self.clip_control(self.to_pid)[...,np.newaxis]
        self.last_error = error
        #pdb.set_trace()
        self.last_pid_call = copy.deepcopy(current_time)
        #print("pid control->", self.to_pid)
        
        x_0 = np.array([self.car_state.x,self.car_state.y,self.car_state.yaw,self.car_state.v_f,self.car_state.v_r,self.car_state.steer_f,self.car_state.steer_r])
        u_0 = copy.deepcopy(self.to_pid)
        solution= scipy.integrate.solve_ivp(self.xdot , t_span=[0,self.sim_dt] ,y0 =x_0,t_eval=[self.sim_dt],args=[u_0])
        X0 = solution.y.reshape((self.sx,1))
        X0[2,0] = self.pi_2_pi(X0[2,0])
        #pdb.set_trace()
        X0 = self.state_clip(X0)
        self.car_state = TemporalState(X0)

    def pid(self,speed, steer, current_speed, current_steer, max_sv, max_a, max_v, min_v):
        """
        Basic controller for speed/steer -> accl./steer vel.

            Args:
                speed (float): desired input speed
                steer (float): desired input steering angle

            Returns:
                accl (float): desired input acceleration
                sv (float): desired input steering velocity
        """
        # steering
        steer_diff = steer - current_steer
        if np.fabs(steer_diff) > 1e-4:
            sv = (steer_diff / np.fabs(steer_diff)) * max_sv
        else:
            sv = 0.0

        # accl
        vel_diff = speed - current_speed
        # currently forward
        if current_speed > 0.:
            if (vel_diff > 0):
                # accelerate
                kp = 10 * max_a / max_v
                accl = kp * vel_diff
            else:
                # braking
                kp = 10.0 * max_a / (-min_v)
                accl = kp * vel_diff
        # currently backwards
        else:
            if (vel_diff > 0):
                # braking
                kp = 2.0 * max_a / max_v
                accl = kp * vel_diff
            else:
                # accelerating
                kp = 2.0 * max_a / (-min_v)
                accl = kp * vel_diff

        return accl, sv







    
    def sim_step(self, u_current,current_time):
        
        u_current= np.reshape(u_current,(self.su,1))
        self.pid_control(u_current, current_time )


    def get_current_states(self):
        st =  self.car_state.get_states(lf=self.lf,lr=self.lr)
        return st
    
    def get_car_vertices(self):
        l = self.Length  # length of mobile robot
        w = self.width   # width of mobile robot
        x = self.car_state.x
        y = self.car_state.y
        psi = self.car_state.yaw
        # Mobile robot coordinates wrt body frame
        #pdb.set_trace()
        mr_co = np.array([[-l/2, l/2, l/2, -l/2],
                        [-w/2, -w/2, w/2, w/2]])
        R_psi = np.array([[np.cos(psi), -np.sin(psi)],
                            [np.sin(psi), np.cos(psi)]])  # rotation matrix
        v_pos = np.dot(np.squeeze(R_psi), np.squeeze(mr_co))  # orientation w.r.t intertial frame              
        v_pos[0,:] = v_pos[0, :] + x
        v_pos[1,:] = v_pos[1, :] + y
        return v_pos

    def pi_2_pi(self,angle):
        while(angle > np.pi):
            angle = angle - 2.0 * np.pi

        while(angle < -np.pi):
            angle = angle + 2.0 * np.pi

        return angle