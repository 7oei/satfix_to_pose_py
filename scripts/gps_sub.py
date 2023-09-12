#!/usr/bin/env python2
# coding: utf-8
from telnetlib import STATUS
import rospy
import sys
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import matplotlib.pyplot as plt
import csv
import tf

POLE_RADIUS = 6356752    # 極半径(短半径)
EQUATOR_RADIUS = 6378137 # 赤道半径(長半径)
E = 0.081819191042815790 # 離心率
#E2= 0.006694380022900788 # 離心率の２乗
E2= 0.006694379990197413546782 # 離心率の２乗再計算

base_lat = 0
base_lon = 0
base_alt = 0
flag = 0
global lat
lat = 0
global lon
lon = 0
global alt
alt = 0
global status
status = 10

def gps_callback(data):
    global lat, lon, alt, status
    lat = data.latitude
    lon = data.longitude
    alt = data.altitude
    status = data.status.status
    
if __name__ == '__main__':
    rospy.init_node('gps_subscriber')
    gps_sub = rospy.Subscriber("ublox/fix", NavSatFix, gps_callback)

    while not rospy.is_shutdown():
        if flag == 0 and lat != 0.00000 and lon != 0.00000 and alt != 0.00000:
            base_lat = lat
            base_lon = lon
            base_alt = alt
            rospy.loginfo("基準lat: %.7f, lon: %.7f, alt: %.7f", lat, lon, alt)

            flag = 1
        if flag >= 1:
            rospy.loginfo("lat: %.7f, lon: %.7f,alt: %.7f", lat, lon, alt)

            lat_per_deg = (2.0*math.pi*EQUATOR_RADIUS)/360; #緯度1度あたりの距離
            lon_per_deg = (2.0*math.pi*EQUATOR_RADIUS*math.cos(lat/180.0*math.pi))/360; #経度1度あたりの距離
            r_x = (lon-base_lon) * lon_per_deg; #ロボット初期位置をもとにグローバル座標系xを求める (Odom_base)
            r_y = (lat-base_lat) * lat_per_deg; #ロボット初期位置をもとにグローバル座標系yを求める (Odom_base)
            r_z = (alt-base_alt)

            pose_pub = rospy.Publisher('/gnss_pose', PoseStamped, queue_size = 10)
            pose_test = PoseStamped()
            pose_test.header.frame_id = 'map'
            pose_test.pose.position.x = r_x
            pose_test.pose.position.y = r_y
            pose_test.pose.position.z = r_z
            pose_test.pose.orientation.z = 0
            pose_pub.publish(pose_test)
            br = tf.TransformBroadcaster()
            br.sendTransform((r_x, r_y, r_z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"gnss_link","map")
        else :
            pose_pub = rospy.Publisher('/gnss_pose', PoseStamped, queue_size = 10)
            pose_test = PoseStamped()
            pose_test.header.frame_id = 'map'
            pose_test.pose.position.x = 0
            pose_test.pose.position.y = 0
            pose_test.pose.position.z = 0
            pose_test.pose.orientation.z = 0
            pose_pub.publish(pose_test)
            br = tf.TransformBroadcaster()
            br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"gnss_link","map")

        rospy.sleep(0.2)