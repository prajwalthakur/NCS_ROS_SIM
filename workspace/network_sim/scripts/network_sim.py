#!/usr/bin/env python3
import numpy as np
from params import MPCParams
from collections import deque
import copy

import rospy
from std_msgs.msg import Header
from vehicle_interface.msg import ControlCommandSeq , ControlCommand , Perception



# color utility
class ColorText:
    def __init__(self):
        # Define ANSI escape codes for styles and colors
        self.bold_text = "\033[1m"
        self.green_color = "\033[32m"
        self.reset_text = "\033[0m"  # Reset text formatting
        self.reset_color = "\033[0m" # Reset color formatting
        self.red_color = "\033[31m"    # Red color







class delay_list_class:
    def __init__(self, msg, timestamp,delay_sampled):
        self.msg = msg
        self.timestamp = timestamp
        self.delay_sampled = delay_sampled
    def __repr__(self):
        # Representation for debugging and logging
        return f"PoseWithTime(pose={self.pose}, timestamp={self.timestamp})"




class network_sim_class():
    def __init__(self):
        MPCParams.__init__(self)
        ColorText.__init__(self)
        rospy.init_node("network_sim_node")
        self.node_init()
        self.perception_feedback_topic_sub = rospy.get_param("~preception_feedback_sub_topic","/perception_feedback")
        self.cmd_control_cloud_sub = rospy.get_param("~cmd_control_cloud","/cmd_control_cloud")
        
        self.perception_subscriber = rospy.Subscriber(self.perception_feedback_topic_sub,Perception,self.perception_cb,queue_size=1)
        self.control_subscriber = rospy.Subscriber(self.cmd_control_cloud_sub,ControlCommandSeq,self.control_cb,queue_size=1)
        
        
        
        self.delayed_perception_feedback_pub = rospy.get_param("~delayed_perception_feedback_pub_topic","/delayed_perception_feedback")
        self.delayed_cmd_control_cloud_pub = rospy.get_param("~delayed_cmd_control_cloud","/delayed_cmd_control_cloud")
        
        self.delayed_perception_pub = rospy.Publisher(self.delayed_perception_feedback_pub,Perception,queue_size=1)
        self.delayed_cmd_control_cloud_pub = rospy.Publisher(self.delayed_cmd_control_cloud_pub,ControlCommandSeq,queue_size=1)
        
        rospy.Timer(rospy.Duration(self.delay_res_dt),callback=self.timer_event)
        
    def node_init(self):
        self.max_len = 200
        self.perception_pose_list = deque(maxlen=self.max_len)
        self.perception_pose_list_copy = deque(maxlen=self.max_len) # DEBUG
        self.last_perception_sent_time = 0.0
        
        self.control_cmd_list = deque(maxlen=self.max_len)
        self.last_control_sent_time = 0.0
        
       
    
    def perception_cb(self,perception_msg):
        #print("perception_cb called")
        current_time = copy.deepcopy(rospy.Time.now().to_sec() )
        delay_sample = self.feedback_delay.sample_delay(size=1)[0]
        assert self.last_perception_sent_time < current_time + delay_sample
        delay_obj = delay_list_class(msg = perception_msg,timestamp=current_time,delay_sampled=delay_sample)
        #print("len of the perception-pose", len(self.perception_pose_list) )
        assert len(self.perception_pose_list)<self.max_len-1
        self.perception_pose_list.append(delay_obj); #self.perception_pose_list_copy.append(delay_obj)
    
    def control_cb(self, control_msg):
        current_time = rospy.Time.now().to_sec() 
        delay_sample = self.input_delay.sample_delay(size=1)[0] 
        assert self.last_control_sent_time < current_time + delay_sample
        delay_obj = delay_list_class(msg = control_msg,timestamp=current_time,delay_sampled=delay_sample)
        self.control_cmd_list.append(delay_obj)
    
    
    def timer_event(self,event):
        current_time = rospy.Time.now().to_sec() 
        # if len(self.perception_pose_list)==0:
        #     print("lenzero")
        if len(self.perception_pose_list)>0:
            top_perception_msg  = self.perception_pose_list[0]
            #print( abs(current_time - (top_perception_msg.timestamp + top_perception_msg.delay_sampled)) )
            if (abs(current_time - (top_perception_msg.timestamp + top_perception_msg.delay_sampled)) < 1e-3) or \
                ((current_time - (top_perception_msg.timestamp + top_perception_msg.delay_sampled)) > 1e-3):
                self.delayed_perception_pub.publish(top_perception_msg.msg)
                #print("msg-popped after " , current_time - (top_perception_msg.timestamp) ) 
                self.perception_pose_list.popleft()
                self.last_perception_sent_time = current_time 
                
                # print(len(self.perception_pose_list))
                #print(f'{self.bold_text}{self.green_color}printing the perception msg after delay ,delay-sample{top_perception_msg.delay_sampled} curr-time - time_Sampled = {current_time- top_perception_msg.timestamp}{self.reset_color}{self.reset_text}')
                
        if len(self.control_cmd_list)>0:
            top_control_msg     = self.control_cmd_list[0]
            if( abs(current_time - (top_control_msg.timestamp + top_control_msg.delay_sampled)) < 1e-3 ) or \
               ((current_time - (top_control_msg.timestamp + top_control_msg.delay_sampled)) > 1e-3 ) :
                #print((current_time - (top_control_msg.timestamp )))
                self.last_control_sent_time = current_time
                self.delayed_cmd_control_cloud_pub.publish(top_control_msg.msg)
                self.control_cmd_list.popleft()
        
        
    def spin(self):
        rospy.loginfo("network_sim_node spinning")
        rospy.spin()
def main():
    network_sim_node = network_sim_class()
    network_sim_node.spin()
if __name__=='__main__':
    main()