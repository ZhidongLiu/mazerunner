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
    escape_route_left = True
    for index in range(len(frontview_scan)):
        if frontview_scan[index] == frontview_scan[len(frontview_scan)-1]:
            break
        elif frontview_scan[index] == 0:
            continue
        elif abs(frontview_scan[index] - frontview_scan[index+1]) > 0.25:
            if frontview_scan[index] - frontview_scan[index+1] > 0.25:
                escape_route_left = False
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
        obstacle_distance = frontview_scan[index]

        adjusted_angle = math.degrees(math.atan(0.2/obstacle_distance))
        escape_angle = obstacle_angle
        if escape_route_left:
            escape_angle = escape_angle + adjusted_angle
        else:
            escape_angle = escape_angle - adjusted_angle

        if escape_angle >= 360:
            escape_angle = escape_angle - 360
        if escape_angle < 0:
            escape_angle = escape_angle + 360

        info.escape_angle = escape_angle
        print "obstacle angle, %s" % obstacle_angle
        print "escape angle, %s" % info.escape_angle
        
        #Escape distance != obstacle distance
        info.escape_distance = obstacle_distance * 1.1 + 0.05
        print "obstacle distance, %s" % obstacle_distance
        print "escape distance, %s" % info.escape_distance
        escape_angle_pub.publish(info)
        obstacle_info_results=[]

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

rospy.init_node('escape_info_publisher')

#Subscribes to scan and publishes to obstacle_angle
escape_angle_pub = rospy.Publisher('escape_info', Escape_info, latch = True)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Wait until ^c
rospy.spin()