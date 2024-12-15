import math
import numpy as np
import bisect
import sys
import os
import time 
from base_classes import RobotHospitalBed
# utility functions
class CubicSpline1D:

    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def calc_first_derivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    
    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

# color utility
class ColorText:
    def __init__(self):
        # Define ANSI escape codes for styles and colors
        self.bold_text = "\033[1m"
        self.green_color = "\033[32m"
        self.reset_text = "\033[0m"  # Reset text formatting
        self.reset_color = "\033[0m" # Reset color formatting
        self.red_color = "\033[31m"    # Red color

# moc control
import pdb
import racingTrack
from findTheta import findTheta
from hpipm_python import *
import scipy
import copy
from  scipy.linalg import block_diag
from params import MPCParams
class mpc_control(MPCParams,ColorText):
    def __init__(self,initialState):
        MPCParams.__init__(self)
        ColorText.__init__(self)
        initialState = initialState.squeeze() 
        self.track_initialization()
        self.Ts = self.control_dt
        self.arc,self.closestIndex,self.last_closestIdx = \
            findTheta(currentPose=np.asarray([initialState[0],initialState[1]]),TrackCenter=self.center_waypoints,theta_coordinates=\
                self.theta_coordinates,trackWidth=self.Trackwidth, last_closestIdx=0 , globalSearch=True)
            
        mpc_current_state = np.array([initialState[0],initialState[1],initialState[2],initialState[3],initialState[4]])    #np.array([initialState[0],initialState[1],initialState[2],initialState[3],initialState[4],self.arc])   
        mpc_current_state = np.reshape(mpc_current_state,(self.ax,1))
        self.x0 =  copy.deepcopy(mpc_current_state)
        self.u0 = np.zeros((self.au,1))
        
        self.u_seqs = np.tile(self.u0,(1,self.horizon_length))
        self.log_itr = 1
        self.attrNames =["_X0","_U0", "_Qk","_qk","_Rk","_Ak","_Bk","_gk","_Ak2" ,"_Bk2",  "_Ck", "_ug","_lg","_lbx","_ubx" , "_lbu","_ubu"]
        
        #self.control_init()  
    def track_initialization(self):
        #self.waypoints_list = self.track_object.waypoints_list
        self.track_object  = racingTrack.racingTrack(self.mpc_track_res,smoothing_distance=0.5, max_width=2.0,circular=True) #racingTrack.racingTrack(1,smoothing_distance=10, max_width=2.0,circular=True)       
        self.Trackwidth = self.track_object.Trackwidth
        self.inner_waypoints,self.center_waypoints,self.outer_waypoints = self.track_object.get_coordinates(self.track_object.dict_waypoints)
        _,self.theta_coordinates,_ = self.track_object.get_s_coord()
        
        _,self.center_waypoints_mpc,_ = self.track_object.get_coordinates_mpc()
        #xy_center = np.vstack((xy_center,np.array([idx,c.x,c.y,c.dist,c.dy,c.dx,c.psi,c.kappa])))
        self.cx = self.center_waypoints_mpc[:,1] 
        self.cy = self.center_waypoints_mpc[:,2]
        self.cyaw =  self.center_waypoints_mpc[:,6]
        self.ck  = self.center_waypoints_mpc[:,7]
        self.s  = self.center_waypoints_mpc[:,3]
        
        # center_xy = self.center_waypoints[:,0:2] #waypoints[:,2:4]
        # ax = center_xy[:,0].tolist()
        # ay =  center_xy[:,1].tolist()        
        # self.cx, self.cy, self.cyaw, self.ck, self.s = calc_spline_course(ax, ay, ds=self.mpc_track_res)   
        self.goal = [self.cx[-1],self.cy[-1]]
        self.cyaw = np.unwrap(self.cyaw)
        self.odelta_f,self.odelta_r, self.oa = None, None, None   #initial control     
        self.attrNames =["_X0","_U0", "_Qk","_qk","_Rk","_Ak","_Bk","_gk","_Ak2" ,"_Bk2", "fk" , "_Ck", "_ug","_lg","_lbx","_ubx" , "_lbu","_ubu"]
        self.sp = self.calc_speed_profile(self.cx,self.cy,self.cyaw,target_speed=self.ref_v)

    def pi_to_pi(self,angle):
        while angle >np.pi:
            angle = angle - 2.0*np.pi
        while angle <np.pi:
            angle = angle + 2.0*np.pi
        return angle

    def calc_speed_profile(self,cx,cy,cyaw,target_speed):
        speed_profile = target_speed*np.ones(len(cx))
        direction = 1.0
        for i in range(len(cx)-1):
            dx = cx[i+1] - cx[i]
            dy = cy[i+1] - cy[i]
            move_direction = np.arctan2(dy,dx).item()
            if dx != 0.0 and dy != 0.0:
                dangle = np.abs(self.pi_to_pi(move_direction-cyaw[i]))
                if dangle >= np.pi/4.0:
                    direction = -1.0
                else:
                    direction = +1.0
            if direction==1.0:
                speed_profile[i] = target_speed
            else:
                speed_profile[i] = target_speed
        return speed_profile
    def calc_nearest_index(self,state,cx,cy,cyaw,pind,N_IND_SEARCH):
        # calculate the nearest index on the trajectory to the current state
        # x,y,yaw,v_f,v_r
        # vf_dot , Vr_dot , steer_f , steer_r        
        # % Calculate the nearest index on the trajectory to the current state
        # % Inputs:
        # %   state - Structure with fields x and y representing the current state
        # %   cx - Vector of x coordinates of the trajectory
        # %   cy - Vector of y coordinates of the trajectory
        # %   cyaw - Vector of yaw angles of the trajectory
        # %   pind - Previous index
        # %   N_IND_SEARCH - Number of points to search around the previous index
        # % Outputs:
        # %   ind - Nearest index on the trajectory
        # %   mind - Distance to the nearest point, signed based on angle
        dx = [state[0] - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
        dy = [state[1] - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = np.sqrt(mind)

        dxl = cx[ind] - state[0]
        dyl = cy[ind] - state[1]

        angle = self.pi_to_pi(cyaw[ind] - np.arctan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind    
    def calc_ref_trajectory(self,state, cx, cy, cyaw, ck, sp, dl, pind):
        state = state.squeeze()
        xref = np.zeros((self.ax, self.horizon_length + 1))
        dref = np.zeros((self.au, self.horizon_length + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind,N_IND_SEARCH=20)
        #pdb.set_trace()
        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2,0]  = cyaw[ind]
        xref[3,0] = sp[ind]
        xref[4,0] = sp[ind]
        travel = 0.0

        for i in range(self.horizon_length + 1):
            travel += abs(state[3]) * self.Ts
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind+dind]
                xref[1, i] = cy[ind+dind]
                xref[2,i]  = cyaw[ind+dind]
                xref[3,i] = sp[ind+dind]
                xref[4,i] = sp[ind+dind]
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2,i]  = cyaw[ncourse - 1]
                xref[3,i] = 0.0
                xref[4,i] = 0.0

        #pdb.set_trace()     
        return xref, ind, dref
    


    def _publish_control(self,state_current , u_seqs : np.ndarray):
        
        current_vf = state_current[3][0]
        current_vr = state_current[4][0]
        #self.get_logger().info('u_Seqs-accl: "%s"' % u_seqs[0,:])   
        mod_u_seqs = copy.deepcopy(u_seqs)
        for i in range(u_seqs.shape[1]):
            current_vf = current_vf + u_seqs[0,i]*self.Ts  # what is the desired speed at Ts + 1
            current_vr = current_vr + u_seqs[1,i]*self.Ts  # what is the desired speed at Ts + 1
            mod_u_seqs[0,i] = current_vf
            mod_u_seqs[1,i] = current_vr
        # with_delay_compensation=False    
        # assert with_delay_compensation==True
        # if with_delay_comepensation==False:
        #mod_u_seqs[:, :] = np.tile(mod_u_seqs[:, 0], (mod_u_seqs.shape[1], 1)).T
            #print("dleay compensation false")
        
        return mod_u_seqs



        
    def control_callback(self, robot_state_obj:RobotHospitalBed,current_time,assumed_input_delay):
        feedback_time = robot_state_obj.robot_state.time_sampled
        #time_current_in_sec = time_current_in_sec
        pred_future_time = current_time + assumed_input_delay

        u_expected_seqs ,  state_predicted   = robot_state_obj.do_forward_sim(feedback_time = feedback_time , current_time = current_time ,predicted_input_delay=assumed_input_delay,do_upsampling=False)

        #u_expected_seqs,state_predicted = robot_state_obj.get_control_seqs_array(),robot_state_obj.get_state_array()
        print(f'{self.bold_text}{self.green_color}u_current={u_expected_seqs.shape}{self.reset_color}{self.reset_text}')
        
        u_current , info_,action_Seq,X_pred =self.step(state_current=state_predicted,u_expected_seqs=u_expected_seqs,
                                                       robot_state_obj=robot_state_obj,future_start_time=pred_future_time)
        
        return u_current , info_,action_Seq,X_pred
    def step(self,state_current:np.ndarray, u_expected_seqs ,robot_state_obj:RobotHospitalBed,future_start_time):
        state_current = state_current.squeeze()
        self.arc,self.closestIndex,self.last_closestIdx = \
            findTheta(currentPose=np.asarray([state_current[0],state_current[1]]),TrackCenter=self.center_waypoints,theta_coordinates=\
                self.theta_coordinates,trackWidth=self.Trackwidth, last_closestIdx=0 , globalSearch=True)     
        
        mpc_current_state =  np.array([state_current[0],state_current[1],state_current[2],state_current[3],state_current[4]]) #np.array([state_current[0],state_current[1],state_current[2],state_current[3],state_current[4],self.arc])   
        mpc_current_state = np.reshape(mpc_current_state,(self.ax,1))
        u_prev = u_expected_seqs[:,0][...,np.newaxis] #self.u_seqs[:,control_itr-1][...,np.newaxis]
        self.u_seqs = u_expected_seqs #np.roll(self.u_seqs,-control_itr)        
        
        xref, self.closestIndex, dref  =     self.calc_ref_trajectory(mpc_current_state, self.cx, self.cy, self.cyaw, self.ck, self.sp, self.mpc_track_res, self.closestIndex)
        print(xref,self.closestIndex,dref)
        cost_list    =  self.create_matrices(xref, mpc_current_state, dref, u_prev)
        X,U,dU,info  =  self.hpipmSolve(cost_list)

        if info["exitflag"]==0:
            self.x_seqs = X
            self.u_seqs = U 
        elif (info["exitflag"]==1):
            pdb.set_trace()
            #self.u_seqs = self.u_seqs[:,1:]
            # exit_flag,U_pred_seqs,X_pred_seqs,track_constraints_pts,costValue,costComponents
        costValue=-1
        costComponents  = np.zeros((1,5))
        
        robot_state_obj._append_control_seqs(control_seqs=self.u_seqs,future_start_time=future_start_time)
        if self.with_pid==True:
            u_seqs = self._publish_control(state_current = mpc_current_state , u_seqs =  copy.deepcopy(self.u_seqs) )
        else:
            u_seqs = copy.deepcopy(self.u_seqs) #self._publish_control(state_current = mpc_current_state , u_seqs =  copy.deepcopy(self.u_seqs) )
        u_current = u_seqs[:,0][:,np.newaxis]
        return u_current , info["exitflag"],u_seqs,self.x_seqs 
    
    def mpc_matrices(self):
            empty_dict = {attr: None for attr in self.attrNames}
            return empty_dict
    
    def create_matrices(self,x_ref_seqs,x0 , dref, u_prev):
        cost_list = []
        u0 = u_prev
        for i in range(self.horizon_length):
            cost_list.append(self.mpc_matrices())
            if i==0:
                cost_list[i]["_X0"] = x0.reshape(self.ax,1)
                cost_list[i]["_U0"] = u0.reshape(self.au,1)
            
            xk = x_ref_seqs[:,i][...,np.newaxis]
            if self.mpc_mode == "zero-input-linearization":
                uk = np.zeros((self.au,1)) # self.u_seqs[:,i]
            elif self.mpc_mode == "constant-input-linearization":
                uk = u_prev
            elif self.mpc_mode == "seq-input-linearization":
                uk = self.u_seqs[:,i][...,np.newaxis]
            cost_list[i]["_Qk"] = self.mpc_costScale*self.generateH(i)
            cost_list[i]["_Rk"] = self.mpc_costScale*self.generateR(i)
            cost_list[i]["_qk"]  = self.mpc_costScale*self.generateq(xk,i)
            #linearized dynamics
            cost_list[i]["_Ak"],cost_list[i]["_Bk"],cost_list[i]["_gk"] =  self.getEqualityConstraints(xk,uk)
            #bounds
            cost_list[i]["_lbx"], cost_list[i]["_ubx"],cost_list[i]["_lbu"], cost_list[i]["_ubu"] = self.getBounds()
        #terminal state
        i = self.horizon_length
        cost_list.append(self.mpc_matrices())
        xk = x_ref_seqs[:,i][...,np.newaxis]
       
        cost_list[i]["_Qk"] =self.mpc_costScale*self.generateH(i)
        cost_list[i]["_qk"]  = self.mpc_costScale*self.generateq(xk,i)
        cost_list[i]["_Rk"] = self.mpc_costScale*2*self.generateR(i)

        #bounds
        cost_list[i]["_lbx"], cost_list[i]["_ubx"],cost_list[i]["_lbu"], cost_list[i]["_ubu"] = self.getBounds()
        return cost_list
    def generateH(self,i):
        if i!=self.horizon_length:
            Q_state = np.diag(self.invTx)@self.Q@np.diag(self.invTx)
            Q_input = np.diag(self.invTu)@self.R@np.diag(self.invTu)
        elif i==self.horizon_length:
            Q_state = self.terminal_cost_scale*np.diag(self.invTx)@self.Q@np.diag(self.invTx)
            Q_input  = self.terminal_cost_scale*self.R
        Qk = 2*block_diag(Q_state,Q_input) + 1e-12*np.eye(self.ax+self.au)
        Qk = 0.5*(Qk + Qk.T)
        return Qk
    
    def generateq(self,xk,i):
        if i!= self.horizon_length:
            Q = self.Q
        elif i==self.horizon_length:
            Q = self.terminal_cost_scale*self.Q
        fx= - 2*xk.T@Q@np.diag(self.invTx)
        fT = np.hstack((fx,np.zeros((1,self.au))))
        f=fT.T
        return f   
    def generateR(self,i):
        if i!=self.horizon_length:
            R = self.Rd
        elif i==self.horizon_length:
            R = self.terminal_cost_scale*self.Rd
        return R    
    def getXdot(self,x0,uk):
        lf = self.lf
        lr = self.lr
        lwb = self.lwb
        

        #x,y,yaw,v_f,v_r
        yaw = x0[2][0]
        
        v_f = x0[3][0]
        v_r = x0[4][0]
        
        v_f_dot = uk[0][0]
        v_r_dot = uk[1][0]
        
        steer_f = uk[2][0]
        steer_r = uk[3][0]
        progress_dot = x0[4][0]
        
        beta = np.arctan2(lr*np.tan(steer_f) + lf*np.tan(steer_r) , lf+lr) #slip angle
        
        xdot = (np.cos(yaw + beta )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / ( 2*np.cos(beta))
        ydot = (np.sin(yaw+beta)*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))) / (2*np.cos(beta))
        yawdot = ( np.tan(steer_f) - np.tan(steer_r) )*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))/(2*(lf+lr))
        
        
        Xdot = np.array([
            [xdot],
            [ydot],
            [yawdot],
            [v_f_dot],
            [v_r_dot],
            [progress_dot]
            ])
        
        return Xdot
    
    def fwd_sim_external(self,x0,uk,dt):
        x0 = x0.squeeze()[0:self.ax]
        x0 = np.reshape(x0,(self.ax,1))
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
        
        x_next = x0 + Xdot*dt
        return x_next
        
    def discitize_linearize_dynamics(self,x0,u0):
        xdot = self.getXdot(x0,u0)
        lf = self.lf
        lr = self.lr
        lwb = self.lwb
        
        yaw = x0[2][0]
        
        v_f = x0[3][0]
        v_r = x0[4][0]
        
        v_f_dot = u0[0][0]
        v_r_dot = u0[1][0]
        
        steer_f = u0[2][0]
        steer_r = u0[3][0]
        #progress_dot = u0[4][0]
      
        A_c = np.array([[0, 0, -(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), \
            np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.cos(steer_f)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.cos(steer_r)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), 0], [0, 0, (v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))*np.cos(steer_f)/(2*(lf + lr)), np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))*np.cos(steer_r)/(2*(lf + lr)), 0], [0, 0, 0, (np.tan(steer_f) - np.tan(steer_r))*np.cos(steer_f)/(2*lf + 2*lr), (np.tan(steer_f) - np.tan(steer_r))*np.cos(steer_r)/(2*lf + 2*lr), 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])

        B_c = np.array([[0, 0, -lr*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_f)**2 + 1)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) + lr*(lf*np.tan(steer_r) + lr*np.tan(steer_f))*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_f)**2 + 1)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) - v_f*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.sin(steer_f)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), -lf*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_r)**2 + 1)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) + lf*(lf*np.tan(steer_r) + lr*np.tan(steer_f))*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_r)**2 + 1)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) - v_r*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.sin(steer_r)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), 0], [0, 0, lr*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_f)**2 + 1)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) + lr*(lf*np.tan(steer_r) + lr*np.tan(steer_f))*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_f)**2 + 1)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) - v_f*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.sin(steer_f)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), lf*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_r)**2 + 1)*np.cos(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) + lf*(lf*np.tan(steer_r) + lr*np.tan(steer_f))*(v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_r)**2 + 1)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)) - v_r*np.sqrt((lf + lr)**2 + (lf*np.tan(steer_r) + lr*np.tan(steer_f))**2)*np.sin(steer_r)*np.sin(yaw + np.arctan2(lf*np.tan(steer_r) + lr*np.tan(steer_f), lf + lr))/(2*(lf + lr)), 0], [0, 0, -v_f*(np.tan(steer_f) - np.tan(steer_r))*np.sin(steer_f)/(2*lf + 2*lr) + (v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(np.tan(steer_f)**2 + 1)/(2*lf + 2*lr), -v_r*(np.tan(steer_f) - np.tan(steer_r))*np.sin(steer_r)/(2*lf + 2*lr) + (v_f*np.cos(steer_f) + v_r*np.cos(steer_r))*(-np.tan(steer_r)**2 - 1)/(2*lf + 2*lr), 0], [1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 1]])
       
        #x,y,yaw,vf,vr,progress = 6
        xdot = xdot[0:self.ax]
        A_c = A_c[0:self.ax,0:self.ax]
        B_c = B_c[0:self.ax,0:self.au]
        #pdb.set_trace()
        gc = xdot- A_c@x0.reshape((self.ax,1)) - B_c@u0.reshape((self.au,1))
        Bc_aug=np.hstack((B_c,gc))
        A_aug = np.vstack((np.hstack((A_c,Bc_aug)),np.zeros((self.ax,self.ax+self.au+1))))    
        #pdb.set_trace()
        tmp_fwd =  scipy.linalg.expm(A_aug*self.Ts)
        
        Ad = np.zeros((self.ax,self.ax))
        Bd = np.zeros((self.ax,self.au))
        gd = np.zeros((self.ax,1))
        
        Ad[0:self.ax,0:self.ax] = tmp_fwd[0:self.ax,0:self.ax]
        Bd[0:self.ax,0:self.au] = tmp_fwd[0:self.ax,self.ax:self.ax+self.au]
        gd[0:self.ax]      =  tmp_fwd[0:self.ax,-1][...,np.newaxis]
        return Ad,Bd,gd       
    
    
    def getEqualityConstraints(self,xk,uk):
        Tx = np.diag(self.Tx)
        Tu = np.diag(self.Tu)
        invTx = np.diag(self.invTx)
        invTu = np.diag(self.invTu)
        Ad,Bd,gd = self.discitize_linearize_dynamics(xk,uk)  
        nx = self.ax 
        nu = self.au
        Ak = np.block([[Tx@Ad@invTx , Tx@Bd@invTu],
                       [np.zeros((nu,nx)) , np.eye(nu)]])
        Bk = np.block([[Tx@Bd@invTu],[np.eye(nu)]])
        gk = np.block([[Tx@gd],[np.zeros((nu,1))]])
        return Ak,Bk,gk
    
    def getBounds(self):
        lbx = self.lbx
        ubx =  self.ubx
        lbu = self.lbu
        ubu =  self.ubu
        return lbx ,ubx  , lbu , ubu
    def hpipmSolve(self,cost_list):
        startTime = time.time()
        codegen_data=0
        env_run  = os.getenv("ENV_RUN")
        if env_run!='true':
            print('ERROR: env.sh has not been sourced! Before executing this example, run:')
            print('source env.sh')
            sys.exit(1)
        nx = self.ax
        nu = self.au
        N = self.horizon_length
        dims = hpipm_ocp_qp_dim(N)
        dims.set('nx', nx+nu , 0 , N)
        dims.set('nu',nu,0,N-1)
        dims.set('nbx',nx+nu,0,N-1) # state bounds
        dims.set('nbu',nu,0,N-1) # input bounds
        dims.set('ng',0,0)
        dims.set('ng',1,1,N) #general polytopic constraints 
        if codegen_data:
            dims.codegen('mpcc.c', 'w')
        qp = hpipm_ocp_qp(dims)
        x0 = np.diag(np.hstack((self.Tx,self.Tu)))@np.vstack((cost_list[0]["_X0"],cost_list[0]["_U0"]))  
        for i in range(N):   
            qp.set('A',cost_list[i]["_Ak"],i)
            qp.set('B',cost_list[i]["_Bk"],i)
            qp.set('b',cost_list[i]["_gk"],i)
            qp.set('Q',cost_list[i]["_Qk"],i)
            qp.set('q',cost_list[i]["_qk"],i)
            qp.set('R',cost_list[i]["_Rk"],i)
        i=N
        qp.set('Q',cost_list[i]["_Qk"],i)
        qp.set('q',cost_list[i]["_qk"],i)  
        
        # set bounds
        for i in range(N+1):
            qp.set('Jbx', np.eye((nx+nu)), i)
            if (i == 0):
                qp.set('lbx', x0, i)
                qp.set('ubx', x0, i)
            else:
                qp.set('lbx', cost_list[i]["_lbx"][0:nx+nu], i)
                qp.set('ubx', cost_list[i]["_ubx"][0:nx+nu], i)
        
            if (i<N):
                qp.set('Jbu', np.eye(nu), i)
                qp.set('lbu', cost_list[i]["_lbu"][0:nu], i)
                qp.set('ubu', cost_list[i]["_ubu"][0:nu], i)
            if (i<N): # defining steering constraints front-steering = - reer-steering
                #pdb.set_trace()
                qp.set('C',np.hstack((np.zeros((nx+nu-2,)) , np.ones((2,)))) , i)
                qp.set('lg',0,i)
                qp.set('ug',0,i)
        if codegen_data:
            qp.codegen('mpcc.c', 'a')    
        # qp sol
        qp_sol = hpipm_ocp_qp_sol(dims)
        #args
        # set up solver arg
        #mode = 'speed_abs'
        #mode = 'speed'
        #mode = 'balance'
        mode = 'balance'
        # create and set default arg based on mode
        arg = hpipm_ocp_qp_solver_arg(dims, mode)
    
        arg.set('mu0', 1e-5)
        arg.set('iter_max', 2000)
        arg.set('tol_stat', 1e-6)
        arg.set('tol_eq', 1e-6)
        arg.set('tol_ineq', 1e-6)
        arg.set('tol_comp', 1e-5)
        arg.set('reg_prim', 1e-12)
        solver = hpipm_ocp_qp_solver(dims, arg)
        solver.solve(qp, qp_sol)
        status = solver.get('status')
        endTime = time.time()
        if status==0:
                    # Log the warning message in red
            print(f'{self.bold_text}{self.green_color}Solver-Success!{self.reset_color}{self.reset_text}')
        else:
            print(f'{self.bold_text}{self.red_color}Solver Failed - Not-Success!:STATUS:{status}{self.reset_color}{self.reset_text}')
            pdb.set_trace()
            #sys.exit()
        u_opt = np.zeros((nu,N))
        x_opt = np.zeros((nx+nu,N+1))

        for i in range(N+1):
            if i<N:
                u_opt[:,i] = (qp_sol.get('u', i)).squeeze()
            x_opt[:,i] = (qp_sol.get('x', i)).squeeze()  

        X = (np.array(x_opt[0:nx,:])*self.invTx[..., np.newaxis]).reshape((nx,N+1))
        U = (np.array(x_opt[nx:,1:])*self.invTu[..., np.newaxis]).reshape((nu,N))
        dU = np.array(u_opt)
        info = {"exitflag":0.06,"QPtime":1}
        if status == 0:
            info["exitflag"] = 0
        else:
            info["exitflag"] = 1
        info["QPtime"] = endTime-startTime
        #pdb.set_trace()
        return  X,U,dU,info
                 
                