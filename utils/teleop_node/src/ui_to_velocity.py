#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Uiteleop():
    def __init__(self):
        self.x_coord = 0
        self.y_coord = 0
        rospy.init_node('ui_listener',anonymous=True)
        rospy.Subscriber('coord',String,self.ui_callback)
        self.cmd_vel_pub = rospy.Publisher('joy/cmd_vel',Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.max_linear_velocity = rospy.get_param('/teleop_node/max_linear_velocity')
        self.max_angular_velocity = rospy.get_param('/teleop_node/max_angular_velocity')
        rospy.spin()

    def ui_callback(self,data):
        '''
        Function that listens to the topic /coord and publishes 
        /joy/cmd_vel (it is not advisable to directly publish values on /cmd_vel)
        '''
        msg = data.data
        msg = json.loads(msg)
        print(msg)
        #NOTE: x corresponds to horizontal motion and y corresponds to vertical motion of the 
        #virtual joystick
        self.x_coord = float(msg['x'])
        self.y_coord = float(msg['y'])
        self.publish_cmd_vel()

    def publish_cmd_vel(self):
        '''
        Publishes cmd_vel to /joy/cmd_vel
        '''
        print("max_linear_velocity:",self.max_linear_velocity)
        move_cmd = Twist()
        move_cmd.linear.x = self.y_coord * (0.01) * self.max_linear_velocity
        move_cmd.angular.z = -(self.x_coord * (0.01) * self.max_angular_velocity)
        self.cmd_vel_pub.publish(move_cmd)
        self.rate.sleep()



if __name__ == '__main__':
    Uiteleop_obj = Uiteleop()