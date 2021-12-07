#!/usr/bin/env python
import numpy as np
import rospy
import json
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


#Constant 
a = 6378137.0 
#Utils
def conv_realtive(latitude, longitude):
    '''
    Convert to relative coordinates using mercartor scale
    '''
    #For testing
    #self.latitude = 13.01
    #self.longitude = 77.57
    #self.altitude = 931
    #Constants used for conversion
    s = np.cos(latitude * np.pi/180)
    x = s * a * (np.pi * longitude/180)
    y = s * a * np.log(np.tan(np.pi*(90 + latitude)/360)) 
    return x,y

class Autonomy():
    def __init__(self):
        rospy.init_node('autonomy')
        self.autonomy_flag = False
        #Get the starting position
        self.start_pos = rospy.get_param('/vehicle_start_pos')
        self.start_x, self.start_y = conv_realtive(self.start_pos[0], self.start_pos[1])
        self.angle = np.deg2rad(rospy.get_param('/angle_offset')) #Offset between vehicle heading and east direction
        self.rotation_matrix = np.array([[np.cos(self.angle), np.sin(self.angle)],
                                         [-np.sin(self.angle), np.cos(self.angle)]])
        self.goal_status = True #Flag to indicate that the vehicle is capable of accepting goals
        # print("self.start_x:",self.start_x)
        # print("self.start_y:",self.start_y)
        #Subscribe to /coord
        rospy.Subscriber('coord',String, self.ui_callback)
        #Read parameters
        self.delta_x = 0.0
        self.delta_y = 0.0
        rospy.spin()

    def ui_callback(self,data):
        msg = data.data
        msg = json.loads(msg)
        self.autonomy_flag = msg['autonomy']
        if(msg['autonomy'] == True and self.goal_status == True):
            latlong = msg['latlong']
            latlong = latlong.split(',')
            latitude = float(latlong[0])
            longitude = float(latlong[1])
            # print("latitude:",latitude)
            # print("longitude:",longitude)
            dest_x, dest_y = conv_realtive(latitude,longitude)
            # print("dest_x:",dest_x)
            # print("dest_y:",dest_y)
            self.delta_x, self.delta_y = self.start_x - dest_x, self.start_y - dest_y
            temp = np.dot(np.array([self.delta_x, self.delta_y]), self.rotation_matrix)
            self.delta_x, self.delta_y = temp[0], temp[1]
            # print("self.delta_x:",self.delta_x)
            # print("self.delta_y:",self.delta_y)
            self.movebase_client()

    def movebase_client(self):
        self.goal_status = False
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.delta_x
        goal.target_pose.pose.position.y = self.delta_y
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal Reached")
            self.goal_status = True
            # return client.get_result()


if __name__ == '__main__':
    try:
        Autonomy_obj = Autonomy()
    except:
        rospy.loginfo("Failed to reach goal position")