#!/usr/bin/env python

import roslib; roslib.load_manifest('cancom')

import rospy
from can_msgs.msg import RadarData

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

from math import pow, degrees, radians
from scipy import mat, cos, sin, arctan, arcsin, sqrt, pi, arctan2, deg2rad, rad2deg

def callback(msg):
    pcloud = PointCloud2()
    pcloud.header.frame_id = '/radar'

    cloud = []
    for ii in range(0,64):
        x = msg.range[ii] * cos(deg2rad(msg.angle[ii]))
        y = msg.range[ii] * sin(deg2rad(msg.angle[ii]))
        z = 0.0
        cloud.append([x,y,z])

    pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)
    
    pc_pub.publish(pcloud)


def main():
    global pc_pub
    rospy.init_node('radar_to_pointcloud')
    
    pc_pub = rospy.Publisher("/radar_pc", PointCloud2,queue_size=100)
    
    rospy.Subscriber("/radar", RadarData, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
