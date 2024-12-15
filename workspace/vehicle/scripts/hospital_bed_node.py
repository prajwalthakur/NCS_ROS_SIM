#!/usr/bin/env python3
import numpy as np
from dataclasses import dataclass
from typing import Optional
from params import MPCParams
from vehicle import car_dynamics_direct_control
from vehicle import car_dynamics_pid 
#import sys
#sys.path.append('/root/workspace/src/vehicle/scripts/')




import rospy
from std_msgs.msg import Header ,Float32
from geometry_msgs.msg import Point ,Twist, Pose , Quaternion , Vector3
from vehicle.msg import Perception , ControlCommand     ,  BedPose , BedFeedback
from tf.transformations import quaternion_from_euler



@dataclass
class RobotStateHospitalBed:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v_x :  float = 0.0
    v_y : float = 0.0
    yaw_rate : float = 0.0
    v_front: float = 0.0
    v_rear: float = 0.0
    steer_front: float = 0.0
    steer_rear: float = 0.0
    time_sampled: Optional[float] = None
    def update_state(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
@dataclass
class RobotControlHospitalBed:
    v_f : float  = 0.0
    v_r : float  = 0.0 
    s_f : float = 0.0
    s_r : float = 0.0
    time_sampled: Optional[float] = None
    def update_control(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)    
    
class hospital_bed_node(MPCParams):
    def  __init__(self):
        
        # ros node initialization
        rospy.init_node('vehicle_node',anonymous=False) 
        MPCParams.__init__(self)
        init_ego_pose_array =self.init_ego_pose
        if self.with_pid==True:
            init_ego_pose_array =self.init_ego_pose
            self.vehicle_obj = car_dynamics_pid.car_model(initial_states=init_ego_pose_array[...,np.newaxis]) 
        else :
            init_ego_pose_array = self.init_ego_pose
            self.vehicle_obj = car_dynamics_direct_control.car_model(initial_states=init_ego_pose_array[...,np.newaxis])             
        
        self.init_states_and_control()
        #rospy.TIme(rospy.Duration(self.sim_dt), self.state_publisher_callback_to_central)
        self.control_subscriber_topic = rospy.get_param("~control_subscriber_topic", "/vehicle_control")
        self.state_publisher_topic = rospy.get_param("~state_publisher_topic", "/ego_state_feedback")
        self.state_publisher =  rospy.Publisher( self.state_publisher_topic,Perception,queue_size=10)
        self.control_subscriber = rospy.Subscriber(self.control_subscriber_topic, ControlCommand , self.control_sub_callback , queue_size=10)
        rospy.Timer(rospy.Duration(self.sim_dt), self.driver_timer_callback)
        rospy.Timer(rospy.Duration(0.005),self.state_publisher_callback)
    def init_states_and_control(self):
        init_control = self.init_controller_control
        self.ego_requested_control =  RobotControlHospitalBed(v_f=init_control[0],v_r=init_control[1] , s_f=init_control[2] , s_r=init_control[3])
        ego_states = self.vehicle_obj.get_current_states()
        self.ego_states = RobotStateHospitalBed(x=ego_states.x,y=ego_states.y,yaw=ego_states.yaw,v_front=ego_states.vf,\
            v_rear=ego_states.vr,steer_front=ego_states.steer_f,steer_rear=ego_states.steer_r,v_x=ego_states.vx , v_y=ego_states.vy,yaw_rate=ego_states.yaw_rate)
        self.control_msg_id = -1
        self.Latency:Float32  = 0.0
        self.CtrlIdx:int = 0       
        
    def control_sub_callback(self,msg):
        #print("called-sub",msg.FSpeed)
        self.ego_requested_control.update_control(v_f=msg.FSpeed,v_r = msg.RSpeed , s_f = msg.FSteer , s_r = msg.RSteer )
        self.control_msg_id = msg.CtrlMsgId
        self.Latency  = msg.Latency
        self.CtrlIdx = msg.CtrlIdx
    def state_publisher_callback(self,event):
        current_time = rospy.Time.now().to_sec() 
        msg  = Perception(
                header = Header(stamp = rospy.Time.now() ,
                        frame_id = "vehicle_feedback_frame"
                        ),
                bed_pose = BedPose(
                            pose = Pose(
                                position = Point(
                                    x = self.ego_states.x,
                                    y = self.ego_states.y,
                                    z = 0.0
                                ),
                            orientation = Quaternion(*quaternion_from_euler(ai=0.0,aj=0.0,ak=self.ego_states.yaw , axes='sxyz'))
                        ),
                twist = Twist(
                    linear = Vector3(
                        x = self.ego_states.v_x,
                        y = self.ego_states.v_y,
                        z = 0.0
                    ),
                    angular = Vector3(
                        x = 0.0 , 
                        y = 0.0 , 
                        z = self.ego_states.yaw_rate
                    )
                        ),
                bed_feedback = BedFeedback(
                    FeedbackFSpeed=self.ego_states.v_front,
                    FeedbackRSpeed=self.ego_states.v_rear,
                    FeedbackFSteer=self.ego_states.steer_front,
                    FeedbackRSteer=self.ego_states.steer_rear,
                    FeedbackCtrlMsgId = self.control_msg_id,
                    FeedbackCtrlIdx = self.CtrlIdx,
                    FeedbackLatency = self.Latency,
                    FeedbackTime =  current_time
                    )   
                )
            )
        self.state_publisher.publish(msg)

    def driver_timer_callback(self,event):
        u_current = np.array([self.ego_requested_control.v_f,self.ego_requested_control.v_r,self.ego_requested_control.s_f,self.ego_requested_control.s_r])
        #print("u_current=>",u_current)
        curr_time = rospy.Time.now().to_sec() 
        self.vehicle_obj.sim_step(u_current, curr_time)
        ego_states = self.vehicle_obj.get_current_states()
        self.ego_states.update_state(x=ego_states.x,y=ego_states.y,yaw=ego_states.yaw,v_front=ego_states.vf,\
            v_rear=ego_states.vr,steer_front=ego_states.steer_f,steer_rear=ego_states.steer_r,v_x=ego_states.vx , v_y=ego_states.vy,yaw_rate=ego_states.yaw_rate)
        
    def spin(self):
        rospy.loginfo("node spinning")
        rospy.spin()


def main():
    vehicle_node = hospital_bed_node()
    vehicle_node.spin()

if __name__ == '__main__':
    main()
