#! /usr/bin/env python
import rospy
import actionlib
import math
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from mazerunner.msg import RotateAction, RotateGoal, RotateResult
from nav_msgs.msg import Odometry

current_rotation = 0
has_turned = False

# Action Request Comes in
def do_move(goal):
    global current_rotation, has_turned
    
    has_turned = False
    relative_angle = 0
    rate = rospy.Rate(10)

    # Converting from angles to radians
    rotation_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move_msg = Twist()

    #If the turn direction is counterclockwise, relative angle is positive
    if goal.counterclockwise:
        relative_angle = goal.degrees_to_rotate*2*math.pi/360 
        move_msg.angular.z = goal.angular_velocity
    else:
        relative_angle = goal.degrees_to_rotate*2*math.pi/360*-1
        move_msg.angular.z = goal.angular_velocity*-1
    print 'Relative Angle, %s' % relative_angle
    print 'Angular velocity, %s' % move_msg.angular.z
    
    # Initialize starting angle
    start_angle = current_rotation
    print 'Starting Angle, %s' % start_angle

    target_angle = start_angle + relative_angle
    if target_angle > math.pi:
        target_angle -= 2*math.pi
    if target_angle < math.pi*-1:
        target_angle += 2*math.pi
    print 'Target Angle, %s' % target_angle


    # Verify success condition
    if goal.counterclockwise:
        if current_rotation < target_angle:
            while current_rotation < target_angle:
                rotation_publisher.publish(move_msg)
                print "Rotation Signal given"
                print "current angle, %s" % current_rotation
                rate.sleep()
        else:
            while current_rotation < target_angle or has_turned == False:
                rotation_publisher.publish(move_msg)
                print "Rotation Signal given"
                print "current angle, %s" % current_rotation
                rate.sleep()
    else: # If clockwise
        if current_rotation > target_angle:
            while current_rotation > target_angle:
                rotation_publisher.publish(move_msg)
                print "Rotation Signal given"
                print "current angle, %s" % current_rotation
                rate.sleep()
        else:
            while current_rotation > target_angle or has_turned == False:
                rotation_publisher.publish(move_msg)
                print "Rotation Signal given"
                print "current angle, %s" % current_rotation
                rate.sleep()


    # Stops the robot once exit condition is met
    print "Stop Signal given"
    print "current angle, %s" % current_rotation
    move_msg.angular.z = 0
    rotation_publisher.publish(move_msg)

    result = RotateResult()
    server.set_succeeded(result, "Rotation Completed")
    time.sleep(2.0)

# Get current orientation
def odometryCb(msg):
    #Euler from Quaternion angles
    global roll, pitch, yaw, current_rotation, has_turned
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #Detect if yaw has gone full circle
    if yaw + math.pi < current_rotation or yaw - math.pi > current_rotation:
        has_turned = True
    current_rotation = yaw

# Declare that we are a node
rospy.init_node('rotate')
rospy.Subscriber('odom', Odometry, odometryCb)

# Declare that this node will handle actions
# When action requests come in, call do_timer method
server = actionlib.SimpleActionServer('rotate', RotateAction, do_move, False)

# Start it up
server.start()

# Wait until ^c
rospy.spin()