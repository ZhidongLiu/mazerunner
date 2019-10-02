#!/usr/bin/env python

import rospy
import math
import statistics
from sensor_msgs.msg import LaserScan
from mazerunner.msg import Escape_info
from std_msgs.msg import Float32, Bool

obstacle_info_results=[]
#stop_publish = False

def scan_callback(msg):
    global obstacle_info_results

    frontview_scan = msg.ranges[270:] + msg.ranges[:90]
    obstacle_angle_found = False
    obstacle_angle = 0
    obstacle_distance = 0
    for index in range(len(frontview_scan)):
        if frontview_scan[index] == frontview_scan[len(frontview_scan)-1]:
            break
        elif frontview_scan[index] == 0:
            continue
        elif abs(frontview_scan[index] - frontview_scan[index+1]) > 0.25:
            obstacle_angle_found = True
            obstacle_angle = index + 270
            if obstacle_angle >= 360:
                obstacle_angle -= 360
            #obstacle_info_results.append(obstacle_angle) #check , need to comment
            break
    #print obstacle_info_results   
    if obstacle_angle_found:
        obstacle_info_results.append(obstacle_angle)

    if len(obstacle_info_results) >= 10:

        info = Escape_info()
        obstacle_angle = find_max_mode(obstacle_info_results)
        #print obstacle_angle
        #Escape angle != obstacle angle
        info.escape_angle = obstacle_angle
        obstacle_distance = frontview_scan[index]
        #Escape distance != obstacle distance
        info.escape_distance = obstacle_distance
        print info.escape_angle
        print info.escape_distance
        escape_angle_pub.publish(info)
        obstacle_info_results=[]
        '''
        while not stop_publish:
            print stop_publish
            print info.escape_angle
            print info.escape_distance
            escape_angle_pub.publish(info)
            obstacle_info_results=[]
        '''
# Find mode of a list
def find_max_mode(list1):
    list_table = statistics._counts(list1)
    len_table = len(list_table)

    if len_table == 1:
        max_mode = statistics.mode(list1)
    else:
        new_list = []
        for i in range(len_table):
            new_list.append(list_table[i][0])
        max_mode = max(new_list) # use the max value here
    return max_mode


#def call_back(msg):
 #   global stop_publish
  #  stop_publish = msg.data

rospy.init_node('escape_info_publisher')

#Subscribes to scan and publishes to obstacle_angle
escape_angle_pub = rospy.Publisher('escape_info', Escape_info, queue_size=1)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

#is_succeed_sub = rospy.Subscriber('needStop', Bool, call_back)

# Wait until ^c
rospy.spin()