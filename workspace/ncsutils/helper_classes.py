from dataclasses import dataclass
import numpy as np
@dataclass
class robotState:
    x:np.float32    = 0.0 # x-coordinate of the car wrt to intertial frame
    y:np.float32    = 0.0 # y-coordinate of the car wrt to inertial frame
    z:np.float32    = 0.0 
    yaw:np.float32  = 0.0 # yaw of the car
    vx:np.float32   = 0.0 # longitudinal speed wrt the car reference frame
    vy:np.float32   = 0.0 # lateral Speed wrt the car reference frame
    vz:np.float32   = 0.0 #
    s_:np.float32    = 0.0 # progress along the reference line
    d:np.float32    = 0.0 # lateral displacement from the ref-line
    msg_sampled_time:np.float32 = 0.0 # time stamp 
    relative_yaw:np.float32 = 0.0


@dataclass
class robotControl:
    vdot:np.float32 = 0.0
    steerAngle : np.float32 = 0.0
    s_dot : np.float32 = 0.0
    msg_sampled_time : np.float32 = 0.0