#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header , Float32
from vehicle_interface.msg import ControlCommandSeq , ControlCommand
from params import MPCParams
import numpy as np
class ColorText:
    def __init__(self):
        # Define ANSI escape codes for styles and colors
        self.bold_text = "\033[1m"
        self.green_color = "\033[32m"
        self.reset_text = "\033[0m"  # Reset text formatting
        self.reset_color = "\033[0m" # Reset color formatting
        self.red_color = "\033[31m"    # Red color
        
class control_interface(MPCParams):
    def __init__(self):
        rospy.init_node("control_interface_node",anonymous=False)
        MPCParams.__init__(self)
        ColorText.__init__(self)
        self.init_buffer()
        self.control_subscriber_topic = rospy.get_param("~cmd_control_cloud_topic","/delayed_cmd_control_cloud") #"/delayed_cmd_control_cloud")
        self.control_publisher_topic = rospy.get_param("~control_pub_topic", "/vehicle_control" )
        
        
        self.control_subsriber = rospy.Subscriber(self.control_subscriber_topic,ControlCommandSeq,self.control_subscriber_callback,queue_size=1)
        #self.control_subsriber = rospy.Subscriber("/delayed_control_command",ControlCommandSeq,self.control_subscriber_callback,queue_size=1)
        
        self.control_publisher = rospy.Publisher(self.control_publisher_topic,ControlCommand,queue_size=1)
        self.actual_input_delay_pub = rospy.Publisher("/actual_input_delay",Float32 , queue_size = 1)
        
        rospy.Timer(rospy.Duration(self.control_dt-1e-3), self.timer_callback)    
        self.control_array = np.zeros((self.au,0))
            
    def init_buffer(self):
            # define control buffer 
            self.buffer_size = 20
            self.control_sampling_time = self.control_dt
            init_control_ref = self.init_control_ref_to_veh
            self.FSpeeds_buffer = [init_control_ref[0]]*self.buffer_size
            self.RSpeeds_buffer = [init_control_ref[1]]*self.buffer_size     
            self.FSteers_buffer = [init_control_ref[2]]*self.buffer_size
            self.RSteers_buffer = [init_control_ref[3]]*self.buffer_size
            self.control_index_in_buffer = 0
            self.ctrl_msg_id = -1
            self.latency  = 0
            current_time_in_seconds = rospy.Time.now().to_sec() 
            self.last_control_msg_recieved_time = current_time_in_seconds     
        
    def timer_callback(self,event):
        self.control_index_in_buffer = self.control_index_in_buffer + 1
        if self.control_index_in_buffer>=self.buffer_size :
            self.control_index_in_buffer = self.buffer_size-1
        FSpeed = self.FSpeeds_buffer[self.control_index_in_buffer]
        RSpeed = self.RSpeeds_buffer[self.control_index_in_buffer]

        FSteer = self.FSteers_buffer[self.control_index_in_buffer]
        RSteer = self.RSteers_buffer[self.control_index_in_buffer]
        
        msg = ControlCommand()
        msg.header = Header(stamp = rospy.Time.now() ,frame_id = "local_controller")
        msg.FSpeed = FSpeed
        msg.RSpeed = RSpeed
        msg.FSteer = FSteer
        msg.RSteer = RSteer
        msg.CtrlMsgId = self.ctrl_msg_id
        msg.Latency = self.latency
        msg.CtrlIdx = self.control_index_in_buffer
        #print(f'{self.bold_text}{self.green_color}publishing the Reference {FSpeed},{RSpeed},{FSteer},{RSteer}{self.reset_color}{self.reset_text}')
        self.control_publisher.publish(msg)
        ctr_at_idx = np.array(([
            [FSpeed],
            [RSpeed],
            [FSteer],
            [RSteer]
        ]))
        self.control_array  = np.hstack((self.control_array,ctr_at_idx))

    # seq-control subscriber callback
    def control_subscriber_callback(self,control_msg):
        # prevents the timer_callback to run when we update the buffer ...
        #: calculate the latency as soon as new msg arrives corresponding to the new msg id
        current_time_in_seconds = rospy.Time.now().to_sec() 
        self.ctrl_msg_id = control_msg.ctrl_msg_id
        self.control_sampling_time  = control_msg.ctrl_sampling_time
        self.FSpeeds_buffer = control_msg.FSpeeds
        self.RSpeeds_buffer = control_msg.RSpeeds
        self.FSteers_buffer = control_msg.FSteers
        self.RSteers_buffer = control_msg.RSteers
        self.latency = current_time_in_seconds - control_msg.ctrl_time_created
        self.actual_input_delay_pub.publish(self.latency)
        self.buffer_size = len(control_msg.FSpeeds)
        #print(f'{self.bold_text}{self.red_color}control-seq-changing , currently executing control-index= {self.control_index_in_buffer} and input latency { self.latency}{self.reset_color}{self.reset_text}')
        #print(f'{self.bold_text}{self.green_color}time-difference b/w reciving two sequence {current_time_in_seconds-self.last_control_msg_recieved_time}{self.reset_color}{self.reset_text}')
        print(f'{self.bold_text}{self.red_color}control-seq-changing , currently executing control-index= {self.control_index_in_buffer} and input latency { self.latency}{self.reset_color}{self.reset_text}')
        print(f'{self.bold_text}{self.green_color}------------------------------------------{self.reset_color}{self.reset_text}') 
        self.control_index_in_buffer = -1
        # print("Update success")
        # self.mutex.release()
        #self.last_control_msg = control_msg
        #self.last_control_msg_time = control_msg["Time"]
        self.last_control_msg_recieved_time = current_time_in_seconds
        #print("last control", self.last_control_msg)
        #print(f'{self.bold_text}{self.red_color}control-seq-changed,latency={ self.latency}{self.reset_color}{self.reset_text}')
        
    def save_control_seq(self):
        print("saved seq")
        np.save("/root/workspace/src/vehicle_interface/scripts/control_seq.npy",self.control_array)

    def spin_node(self):
        rospy.loginfo(" control interface node spinning")
        rospy.on_shutdown(self.save_control_seq)
        rospy.spin()
def main():
    control_interface_node = control_interface()
    control_interface_node.spin_node()
if __name__=='__main__':
    main()