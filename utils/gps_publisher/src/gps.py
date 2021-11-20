#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf 
from nav_msgs.msg import Odometry

import serial
import numpy as np
import math
from sys import exit

class Readgps():
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        #Constants used for conversion
        self.a = 6378137.0 #Radius of the earth
        #Rospy publisher
        rospy.init_node('gps_publisher')
        self.pub = rospy.Publisher('gps_odom',Odometry,queue_size=50)
        self.rate = rospy.Rate(1.0)

    def read_gps(self):
        '''
        Read Lat, Long and altitude values
        '''
        ser = serial.Serial('/dev/ttyUSB2', 4800, timeout=5) #TODO: Write udev rules for this
        while not rospy.is_shutdown():
            try:
                line = ser.readline()
                line = line.decode('utf-8')
                splitline = line.split(',')
                if splitline[0] == '$GPGGA':
                    lat = splitline[2]
                    lat_deg = lat[:2] 
                    lat_min = lat[2:] 

                    latdirection = splitline[3]
                    
                    lon = splitline[4]
                    lon_deg = lon[:3].lstrip("0")
                    lon_min = lon[3:]

                    londirection = splitline[5]
                    
                    # self.latitude = lat_deg +' deg ' + lat_min +"'" + latdirection
                    # self.longitude = lon_deg + ' deg ' + lon_min + "'" + londirection

                    # self.altitude = splitline[9]

                    # print('lat:', self.latitude)
                    # print('lon:', self.longitude)
                    # print('altitude(M):',self.altitude)
                    self.latitude = int(lat_deg) + float(lat_min)/60
                    self.longitude = int(lon_deg) + float(lon_min)/60
                    self.altitude = float(splitline[9])
                    #Convert to xyz
                    self.conv_realtive()
                    #Publish
                    self.publish()
                    self.rate.sleep()
            except Exception as e:
                print('error', e)
            # except KeyboardInterrupt:
                # exit()
    

    def conv_realtive(self):
        '''
        Convert to relative coordinates using mercartor scale
        '''
        #For testing
        #self.latitude = 13.01
        #self.longitude = 77.57
        #self.altitude = 931
        #Constants used for conversion
        s = np.cos(self.latitude * np.pi/180)
        self.x = s * self.a * (np.pi*self.longitude/180)
        self.y = s * self.a * np.log(np.tan(np.pi*(90 + self.latitude)/360)) 
        self.z = self.altitude
    

    def publish(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'base_link'
        #Position
        odom.child_frame_id = 'gps'
        odom.pose.pose.position.x = self.x 
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        #publish
        self.pub.publish(odom)



if __name__ == '__main__':
    Readgpsobj = Readgps()
    Readgpsobj.read_gps()




