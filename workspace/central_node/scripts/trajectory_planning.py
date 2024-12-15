#!/usr/bin/env python3
import numpy as np
from mpc_controller.mpc_main_normalized import mpc_control
import base_classes
import copy

import rospy
from params import MPCParams
from std_msgs.msg import Float32 , Header
from central_node.msg import Perception , ControlCommandSeq , MpcPrediction
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ColorText:
    def __init__(self):
        # Define ANSI escape codes for styles and colors
        self.bold_text = "\033[1m"
        self.green_color = "\033[32m"
        self.reset_text = "\033[0m"  # Reset text formatting
        self.reset_color = "\033[0m" # Reset color formatting
        self.red_color = "\033[31m"    # Red color

class trajectory_planning_class():
    def __init__(self):
        rospy.init_node("trajectory_planning_node")
        MPCParams.__init__(self)
        ColorText.__init__(self)
        self.node_init()    
        self.perception_topic = rospy.get_param("~perception_topic","/delayed_perception_feedback")
        self.control_topic = rospy.get_param("~control_topic","/cmd_control_cloud")
        self.mpc_pred_pub_topic = rospy.get_param("~mpc_pred_topic","/mpc_pred_pose")
        self.perception_subscriber = rospy.Subscriber(self.perception_topic,Perception,self.perception_cb , queue_size=1)
        self.control_publisher = rospy.Publisher(self.control_topic,ControlCommandSeq,queue_size=1)
        
        self.mpc_pred_pub = rospy.Publisher(self.mpc_pred_pub_topic,MpcPrediction,queue_size=1)
        
        
        self.predicted_input_delay_pub = rospy.Publisher("/predicted_input_delay",Float32, queue_size= 1   )
        self.actual_feedback_delay = rospy.Publisher("/actual_feedback_delay",Float32,queue_size=1)
        self.predicted_feedback_delay = rospy.Publisher("/predicted_feedback_delay",Float32,queue_size=1)
        self.mpc_itr=0
        rospy.Timer(rospy.Duration(self.input_mean_delay), self.mpc_callback)    
    def node_init(self):
        init_ego_pose = self.init_ego_pose
        self.mpc_controller = mpc_control(initialState=init_ego_pose)
        curr_time = copy.deepcopy(rospy.Time.now().to_sec())
        if self.do_upsampling :
            control_update_rate = 2*(1/self.control_dt)
            seq_length  = 2*self.horizon_length
            self.robot_obj =\
                base_classes.RobotHospitalBed(vehicle_type="hospital_bed",current_time=curr_time,\
                    control_seq_length=seq_length,control_update_rate=control_update_rate,do_upsampling=self.do_upsampling)
        else : 
            control_update_rate = (1/self.control_dt)
            seq_length  = self.horizon_length
            self.robot_obj =\
                base_classes.RobotHospitalBed(vehicle_type="hospital_bed",current_time=curr_time,\
                    control_seq_length=seq_length,control_update_rate=control_update_rate,do_upsampling=self.do_upsampling)           
        
        self.perception_msg = None
        self.feedback_delay = 0.0
        self.estimated_delay  = 0.0
        self.last_mpc_call_time = 0.0
        self.ctrl_msg_id_last_sent = 0
        self.computed_control_commands = np.zeros((2,100))
        self.prev_control_sequences = np.zeros((2,100))
    
    def perception_cb(self,perception_msg):
        current_time = copy.deepcopy(rospy.Time.now().to_sec())
        self.perception_msg = copy.deepcopy(perception_msg)
        time_diff = (current_time - perception_msg.bed_pose.bed_feedback.FeedbackTime ) #self.perception_msg.header.stamp.to_sec() ) #perception_msg.bed_pose.bed_feedback.FeedbackTime)  #- self.prev_feedback_time
        self.feedback_delay = time_diff
        msg_id_feedback = perception_msg.bed_pose.bed_feedback.FeedbackCtrlMsgId
        # if (msg_id_feedback==-1 ) or (msg_id_feedback == self.ctrl_msg_id_last_sent):    
        #     latency_obs = perception_msg.bed_pose.bed_feedback.FeedbackLatency
        #self.actual_input_delay_pub.publish(time_diff)
        ############## mpc call from perception ###############
        delay_predicted = self.feedback_delay + self.input_mean_delay
        self.predicted_input_delay_pub.publish(self.input_mean_delay)
        self.actual_feedback_delay.publish(self.feedback_delay)
        self.predicted_feedback_delay.publish(self.feedback_mean_delay)
        print(f'{self.bold_text}{self.green_color}feedback-delay{self.feedback_delay}{self.reset_color}{self.reset_text}')
        #start_time = rospy.Time.now() 
        #self.call_mpc(assumed_input_delay= self.input_mean_delay)
        #computational_time = rospy.Time.now() - start_time
        
    def mpc_callback(self,event):
        self.call_mpc(assumed_input_delay= self.input_mean_delay)
            
    def call_mpc(self,assumed_input_delay):
        current_time = rospy.Time.now().to_sec()
        perception_msg = self.perception_msg
        
        rospy.loginfo("=========== calling MPC =========")
        ego_bed_pose = perception_msg.bed_pose
        X = ego_bed_pose.pose.position.x
        Y = ego_bed_pose.pose.position.y
        theta = euler_from_quaternion([ego_bed_pose.pose.orientation.x, ego_bed_pose.pose.orientation.y, ego_bed_pose.pose.orientation.z, ego_bed_pose.pose.orientation.w])[2]
        Vx = ego_bed_pose.twist.linear.x
        Vy = ego_bed_pose.twist.linear.y
        feedback_msg = ego_bed_pose.bed_feedback
        Vf = feedback_msg.FeedbackFSpeed
        Vr = feedback_msg.FeedbackRSpeed
        # Use feedback steer angle
        Sf = feedback_msg.FeedbackFSteer
        Sr = feedback_msg.FeedbackRSteer
        feedback_time = perception_msg.bed_pose.bed_feedback.FeedbackTime
        msg_id_feedback =  perception_msg.bed_pose.bed_feedback.FeedbackCtrlMsgId
        control_idx_in_seq = perception_msg.bed_pose.bed_feedback.FeedbackCtrlIdx
        # update the state of the robot with the feedback
        #print(f'{self.bold_text}{self.red_color}feedback-time{feedback_time},time_current_in_sec{time_current_in_sec},diff={feedback_time-time_current_in_sec}{self.reset_color}{self.reset_text}')
        self.robot_obj._update_state(x=X,y=Y,yaw=theta,v_x=Vx,v_y=Vy,v_front=Vf,v_rear=Vr,steer_front = Sf, steer_rear=Sr,time_sampled=feedback_time,msg_id = msg_id_feedback , control_idx_in_seq = control_idx_in_seq)
        #rospy.signal_shutdown("MPC operation completed.")
        #robot_state_obj:RobotHospitalBed,current_time,assumed_input_delay, with_delay_comepensation
        _,_,action_Seq,X_pred = self.mpc_controller.control_callback(self.robot_obj,current_time=current_time,assumed_input_delay=assumed_input_delay)
        # if self.mpc_itr==1 : 
        #rospy.signal_shutdown("MPC operation completed.")  
        self.pub_control_seq(action_Seq,X_pred)        
        self.mpc_itr+=1
    def pub_control_seq(self,action_Seq,X_pred):
        control_msg = ControlCommandSeq()
        control_msg.header = Header(stamp = rospy.Time.now() ,
                        frame_id = "trajectory_planning"
                        )
        print(action_Seq[0,:])
        control_msg.FSpeeds =  action_Seq[0,:].tolist() 
        control_msg.RSpeeds =  action_Seq[1,:].tolist() 
        control_msg.FSteers =  action_Seq[2,:].tolist() 
        control_msg.RSteers =  action_Seq[3,:].tolist() 
        control_msg.ctrl_time_created = rospy.Time.now().to_sec()
        unique_id = (rospy.Time.now().to_sec()% 10000)
        control_msg.ctrl_msg_id  =  unique_id
        self.ctrl_msg_id_last_sent = unique_id
        control_msg.ctrl_sampling_time  = self.control_dt
        self.control_publisher.publish(control_msg)
        
        prediction_msg = MpcPrediction()
        prediction_msg.header = Header(stamp = rospy.Time.now() ,
                        frame_id = "trajectory_planning"
                        )
        prediction_msg.ctrl_msg_id = unique_id 
        
        prediction_msg.x_pose   = X_pred[0,:].tolist()
        prediction_msg.y_pose   = X_pred[1,:].tolist()
        prediction_msg.yaw_pose = X_pred[2,:].tolist()
        
        self.mpc_pred_pub.publish(prediction_msg)

        
    def spin(self):
        rospy.loginfo("spinning trajectory_planning node")
        rospy.spin() 


def main():
    trajectory_planning_node = trajectory_planning_class()
    trajectory_planning_node.spin()

        
if __name__=='__main__':
    main()