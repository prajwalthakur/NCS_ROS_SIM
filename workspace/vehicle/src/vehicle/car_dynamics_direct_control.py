# this is the true model of the car
#!/usr/bin/env python3 
import numpy as np
from params import MPCParams
import copy
from dataclasses import dataclass

@dataclass
class robotstate:    
    x:np.float32
    y:np.float32
    yaw:np.float32
    vf : np.float32
    vr : np.float32
    steer_f  = 0.0
    steer_r = 0.0
    vx  = 0.0
    vy  = 0.0
    yaw_rate = 0.0


class TemporalState:
    def __init__(self, X0):
        self.x = X0[0,0]
        self.y = X0[1,0]
        self.yaw = X0[2,0]
        self.v_f  =   X0[3,0]
        self.v_r  =   X0[4,0]
        self.members = ['x', 'y','yaw','v_f','v_r','steer_f','steer_r', 'progress']
    def get_states(self):
        state = robotstate(x=self.x,y=self.y,yaw=self.yaw,vf =self.v_f,vr=self.v_r)
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
        self.min_control_array = np.array([self.car_min_vf_dot, self.car_min_vr_dot ,self.car_min_steering_f,self.car_min_steering_r])[...,np.newaxis]
        self.max_control_array = -copy.deepcopy(self.min_control_array)
        self.min_car_state = np.array([-np.inf,-np.inf,-np.inf,self.car_min_vf, self.car_min_vr ])[...,np.newaxis]
        self.max_car_state = - copy.deepcopy(  self.min_car_state ) 

        self.boundaryTrackConstraint_left = np.zeros((1,3))
        self.boundaryTrackConstraint_right = np.zeros((1,3))
        self.opt_predictionPoints = np.zeros((1,3))
        self.mpcPredictionPoints = np.zeros((1,3))
        self.timeElapsed = 0
        self.mpcCostCOmponents = 0
        self.current_time=0.0
        self.sx = 5
        self.su = 4
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
        steer_f = u[2][0]
        steer_r = u[3][0]
        
        x_ = x[0]
        y_ = x[1]
        yaw = x[2]
        v_f = x[3]
        v_r = x[4]

        
        beta = np.arctan2(self.lr*np.tan(steer_f) + self.lf*np.tan(steer_r) , self.lf+self.lr) #slip angle
        
        xdot = (np.cos(yaw + beta )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / ( 2*np.cos(beta))
        ydot = (np.sin(yaw+beta)*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / (2*np.cos(beta))
        yawdot = ( np.tan(steer_f) - np.tan(steer_r) )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))/(2*(self.lf+self.lr))
        
        Xdot = np.array([
            [xdot],
            [ydot],
            [yawdot],
            [vf_dot],
            [vr_dot],
            ])    
        return Xdot.squeeze()
    
    def clip_control(self,control):
        control = np.clip(control, self.min_control_array , self.max_control_array)
        return control
    def state_clip(self,state):
        state  = np.clip(state,self.min_car_state,self.max_car_state)
        return state
    
    def direct_control(self,control , current_time):
   
        x_0 = np.array([self.car_state.x,self.car_state.y,self.car_state.yaw,self.car_state.v_f,self.car_state.v_r])
        u_0 = copy.deepcopy(control)
        #solution= scipy.integrate.solve_ivp(self.xdot , t_span=[0,self.sim_dt] ,y0 =x_0,t_eval=[self.sim_dt],args=[u_0])
        X0 = x_0 + self.xdot(current_time,x_0,u_0)*self.sim_dt #solution.y.reshape((self.sx,1))
        X0[2] = self.pi_2_pi(X0[2])
        X0 = self.state_clip(X0[...,np.newaxis])
        self.car_state = TemporalState(X0)
    
    def sim_step(self, u_current,tt=None):
        
        u_current= np.reshape(u_current,(self.su,1))
        self.current_time  += self.sim_dt
        self.direct_control(u_current, self.current_time  )


    def get_current_states(self):
        st =  self.car_state.get_states()
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