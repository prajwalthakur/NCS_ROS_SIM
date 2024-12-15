#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from vehicle_interface.msg import ControlCommandSeq , ControlCommand , Perception
from params import MPCParams
class feedback_interface(MPCParams):
    def __init__(self):
        rospy.init_node("feedback_interface_node",anonymous=False)
        MPCParams.__init__(self)
        self.init_perception_msg()
        self.feedback_subscriber_topic = rospy.get_param("~state_subscriber_topic", "/ego_state_feedback")
        self.feedback_publisher_topic = rospy.get_param("~perception_publisher_topic", "/perception_feedback" )
        self.feedback_subsriber = rospy.Subscriber(self.feedback_subscriber_topic,Perception,self.feedback_sub_cb,queue_size=1)
        self.feedback_publisher = rospy.Publisher(self.feedback_publisher_topic,Perception,queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.sensor_dt), self.timer_callback)    
     
    def init_perception_msg(self):
        self.pereption_msg = Perception()
        self.pereption_msg.header =  Header(stamp = rospy.Time.now() ,
                        frame_id = "perception_frame")   
    def feedback_sub_cb(self,msg):
        self.pereption_msg = msg
    def timer_callback(self,event):
        self.pereption_msg.header =  Header(stamp = rospy.Time.now() ,
                        frame_id = "perception_frame")
        self.feedback_publisher.publish(self.pereption_msg) 
    def spin_node(self):
        rospy.loginfo(" feedback interface node spinning")
        rospy.spin()
def main():
    feedback_interface_node = feedback_interface()
    feedback_interface_node.spin_node()
if __name__=='__main__':
    main()