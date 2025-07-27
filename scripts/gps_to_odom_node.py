#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
import pyproj

class GpsToOdom:
    def __init__(self):
        rospy.init_node('gps_to_odom_converter')
        
        # Initialize UTM projection
        self.proj = pyproj.Proj(proj='utm', zone=51, ellps='WGS84')  # Adjust zone based on your location
        
        # Get parameters
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        
        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher('/gps/odometry', Odometry, queue_size=10)
        self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        
        # Store initial position
        self.init_x = None
        self.init_y = None

    def gps_callback(self, msg):
        if msg.status.status == -1:  # Check if GPS fix is valid
            return

        # Convert lat/lon to UTM
        x, y = self.proj(msg.longitude, msg.latitude)
        
        # Initialize reference point if not set
        if self.init_x is None:
            self.init_x = x
            self.init_y = y
        
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        # Set position relative to initial point
        odom.pose.pose.position.x = x - self.init_x
        odom.pose.pose.position.y = y - self.init_y
        odom.pose.pose.position.z = msg.altitude if msg.altitude != 0 else 0.0
        
        # Set covariance based on GPS accuracy
        odom.pose.covariance = [msg.position_covariance[0], 0, 0, 0, 0, 0,
                               0, msg.position_covariance[4], 0, 0, 0, 0,
                               0, 0, msg.position_covariance[8], 0, 0, 0,
                               0, 0, 0, 99999, 0, 0,
                               0, 0, 0, 0, 99999, 0,
                               0, 0, 0, 0, 0, 99999]
        
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        converter = GpsToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass