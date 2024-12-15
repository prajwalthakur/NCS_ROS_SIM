#!/usr/bin/env python3
import numpy as np
import matplotlib
#print(matplotlib.get_backend())  # Should output "QtAgg"
# matplotlib.use("Agg")  # Explicitly set the backend
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches
from params import MPCParams 
from SimParameters import CarParams
from network_sim.msg import Perception , MpcPrediction
from geometry_msgs.msg import PoseArray
import racingTrack
import rospy
from tf.transformations import euler_from_quaternion
import threading



class vehicle_sim_class():
    def __init__(self):
        rospy.init_node("vehicle_sim_node",anonymous=False)
        MPCParams.__init__(self)
        CarParams.__init__(self)
        self.node_init()
        self.state_sub_topic  =  rospy.get_param("~state_subscriber_topic", "/ego_state_feedback")
        self.mpc_pred_pub_topic = rospy.get_param("~mpc_pred_topic", "/mpc_pred_pose")
        self.state_sub        =  rospy.Subscriber(self.state_sub_topic,Perception,callback=self.state_sub_cb,queue_size=10)
        self.mpc_pred_sub = rospy.Subscriber(self.mpc_pred_pub_topic,MpcPrediction,callback = self.mpc_pred_cb,queue_size=5)
        self.reference_path = rospy.Subscriber("/reference_path",PoseArray , callback=self.ref_path_pub , queue_size=1)
        
        #self.mpc_info_sub = rospy.Subscriber(self.mpc_info_sub , )
        self.timer = rospy.Timer(rospy.Duration(self.plot_update_dt),self.plot_cb)
        self.plot_lock = threading.Lock()
        

    def node_init(self):
        self.paused = False
        self.track_object  = racingTrack.racingTrack(self.mpc_track_res,smoothing_distance=0.5, max_width=2.0,circular=True) #racingTrack.racingTrack(1,smoothing_distance=10, max_width=2.0,circular=True)       
        self.robot_fig , self.robot_axis = plt.subplots()
        self.robot_axis.set_xlim( -80, 80  )
        self.robot_axis.set_ylim( -80 , 80 )
        self.robot_axis.set_aspect('equal')
        self.robot_fig.tight_layout()
        self.plot_track(self.robot_axis , self.robot_fig)
        self.car_L = self.model_parameters["L"]
        self.car_W = self.model_parameters["W"]
        self.ego_x_pose,self.ego_y_pose,self.ego_yaw = self.init_ego_pose[0],self.init_ego_pose[1],self.init_ego_pose[2]


        vertex_x , vertex_y  = self.calc_vertex()        
        vertex_directions = np.hstack((np.array(vertex_x).reshape((4,1)),np.array(vertex_y).reshape((4,1))))
        #vertex_directions = np.array([[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]])
        self.robot_state = patches.Polygon(
                vertex_directions,
                alpha=1.0,
                closed=True,
                fc='b',
                ec="None",
                zorder=10,
                linewidth=2,
            )

        # Add grid
        self.robot_axis.grid(
            True,  # Enable grid
            which='both',  # Major and minor grid lines
            linestyle='--',  # Dashed lines
            linewidth=0.5,  # Line width
            alpha=0.7  # Transparency of grid lines
        )
        self.robot_axis.add_patch(self.robot_state)        
    def ref_path_pub(self, msg):
        """
        Callback function for the reference path (PoseArray).
        Extracts x, y positions from the poses and stores them.
        """
        self.ref_x = [pose.position.x for pose in msg.poses]
        self.ref_y = [pose.position.y for pose in msg.poses]
        rospy.loginfo(f"Received {len(msg.poses)} poses in the reference path.")

    def state_sub_cb(self,msg):
        pose_ =  msg.bed_pose.pose.position
        self.ego_x_pose = pose_.x
        self.ego_y_pose = pose_.y
        orientation = msg.bed_pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        _, _, self.ego_yaw = euler_from_quaternion(quaternion)
    
    def mpc_pred_cb(self,msg):
        self.x_predictions = msg.x_pose
        self.y_prediction  = msg.y_pose
        self.yaw_prediction = msg.yaw_pose
    
    def plot_cb(self,event):
        if not self.paused:
            #self.data_axis.xax
            #is.set_major_locator(MultipleLocator(0.01))
            self.camera_center = [self.ego_x_pose,self.ego_y_pose]

            vertex_x , vertex_y  = self.calc_vertex()
            
            self.robot_state.set_xy( np.array([vertex_x, vertex_y]).T  )
            self.camera_center = [self.ego_x_pose, self.ego_y_pose]
            # Update plot limits based on camera center and car dimensions
            buffer = 5 # Add a buffer around the car for better visibility
            xlim_min = self.camera_center[0] - self.car_L/2 - 2*buffer
            xlim_max = self.camera_center[0] + self.car_L/2 + 2*buffer
            ylim_min = self.camera_center[1] - self.car_W/2 - buffer
            ylim_max = self.camera_center[1] + self.car_W/2 + buffer
            self.robot_axis.set_xlim(xlim_min, xlim_max)
            self.robot_axis.set_ylim(ylim_min, ylim_max)
            # Update Plots
            # print("before error")
            # print(type(self.robot_fig.canvas))
            

            if hasattr(self, "ref_x") and hasattr(self, "ref_y"):
                self.robot_axis.plot(self.ref_x, self.ref_y) #'m--', label='Reference Path', zorder=5)
                self.robot_axis.legend()
            self.ref_plotted  = True

            self.robot_fig.canvas.draw()

    def calc_vertex(self):
        l = self.car_L / 2
        w = self.car_W / 2
        vertex_x = [
            self.ego_x_pose + l * np.cos(self.ego_yaw) - w * np.sin(self.ego_yaw),
            self.ego_x_pose + l * np.cos(self.ego_yaw) + w * np.sin(self.ego_yaw),
            self.ego_x_pose - l * np.cos(self.ego_yaw) + w * np.sin(self.ego_yaw),
            self.ego_x_pose - l * np.cos(self.ego_yaw) - w * np.sin(self.ego_yaw),
        ]
        vertex_y = [
                    self.ego_y_pose + l * np.sin(self.ego_yaw) + w * np.cos(self.ego_yaw),
                    self.ego_y_pose + l * np.sin(self.ego_yaw) - w * np.cos(self.ego_yaw),
                    self.ego_y_pose - l * np.sin(self.ego_yaw) - w * np.cos(self.ego_yaw),
                    self.ego_y_pose - l * np.sin(self.ego_yaw) + w * np.cos(self.ego_yaw),
                ]
        return vertex_x , vertex_y

    def set_lines(self,col,waypoints, ax , fig,line_size=1 ):
        track_line =  lines.Line2D([],[],linestyle='--',color = col ,linewidth=line_size)
        track_line.set_data(waypoints[:,0],waypoints[:,1])
        ax.add_line(track_line)  
        
    def plot_track(self,ax,fig):
        waypoints = self.track_object.dict_waypoints
        waypoints_inner,waypoints_center,waypoints_outer = self.track_object.get_coordinates(waypoints)                 
        range_waypoints = len(waypoints_center)-1
        self.set_lines('b',waypoints_inner , ax , fig )
        self.set_lines('r',waypoints_center, ax , fig ,line_size=0.4)
        self.set_lines('g',waypoints_outer, ax , fig )
                
    def spin(self):
        rospy.loginfo("vehicle_sim_node spinning")
        plt.show()

def main():
    plt.switch_backend("Qt4Agg")
    vehicle_sim_node = vehicle_sim_class()
    vehicle_sim_node.spin()

if __name__=='__main__':
    main()