#!/usr/bin/env python

import rospy
import math
import statistics
from sensor_msgs.msg import LaserScan
from mazerunner.msg import Escape_info
from std_msgs.msg import Float32, Bool

obstacle_info_results=[]
no_escape_route_counter = 0
#stop_publish = False

def scan_callback(msg):
    global obstacle_info_results, no_escape_route_counter

    frontview_scan = msg.ranges[270:] + msg.ranges[:90]


    obstacle_angle_found = False
    obstacle_angle = 0
    obstacle_distance = 0
    escape_route_left = True
    info = Escape_info()

    for index in range(len(frontview_scan)):
        last_index = len(frontview_scan) - 1
        if index == last_index:
            no_escape_route_counter = no_escape_route_counter + 1
            print "counter, %s" %  no_escape_route_counter
            break
        elif frontview_scan[index] == 0:
            continue
        elif abs(frontview_scan[index] - frontview_scan[index+1]) > 0.3 and frontview_scan[index+1] != 0:
            if frontview_scan[index] - frontview_scan[index+1] > 0.3:
                escape_route_left = False
            obstacle_angle_found = True
            obstacle_angle = index + 270
            if obstacle_angle >= 360:
                obstacle_angle -= 360
            break

    #print obstacle_info_results   
    if obstacle_angle_found:
        obstacle_info_results.append(obstacle_angle)

    if no_escape_route_counter > 10:
        info.escape_possible = False
        escape_angle_pub.publish(info)
        obstacle_info_results = []
        no_escape_route_counter = 0

    elif len(obstacle_info_results) >= 10:
        info.escape_possible = True

        obstacle_angle = find_max_mode(obstacle_info_results)

        obstacle_distance = 0
        if escape_route_left:
            obstacle_distance = frontview_scan[index]
        else:
            obstacle_distance = frontview_scan[index+1]

        adjusted_angle = math.degrees(math.atan(0.32/obstacle_distance))
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
        
        info.escape_distance = math.sqrt(math.pow(0.32, 2) + math.pow(obstacle_distance, 2)) + 0.25
        print "obstacle distance, %s" % obstacle_distance
        print "escape distance, %s" % info.escape_distance

        escape_angle_pub.publish(info)
        obstacle_info_results = []
        no_escape_route_counter = 0
    

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
escape_angle_pub = rospy.Publisher('escape_info', Escape_info, queue_size=1)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Wait until ^c
rospy.spin()