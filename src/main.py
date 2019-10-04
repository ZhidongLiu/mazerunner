#! /usr/bin/env python
import rospy
import math
import time
import actionlib
from geometry_msgs.msg import Twist
from mazerunner.msg import Escape_info
from mazerunner.msg import RotateAction, RotateGoal, RotateResult, RotateFeedback
from mazerunner.msg import MoveForwardAction, MoveForwardGoal, MoveForwardResult, MoveForwardFeedback


rospy.init_node('Main')

def callback(msg):   

    twist = Twist()
    pub.publish(twist)

    rospy.loginfo("Waiting for server...")
    rotate_client.wait_for_server()
    move_forward_client.wait_for_server()

    rotate_goal = RotateGoal()
    move_forward_goal = MoveForwardGoal()

    move_forward_goal.distance = msg.escape_distance
    move_forward_goal.linear_velocity = 0.18

    rotate_goal.angular_velocity = math.pi/12

    if msg.escape_possible:

        if msg.escape_angle >= 270:
            rotate_goal.degrees_to_rotate = 360 - msg.escape_angle
            rotate_goal.counterclockwise = False
        else:
            rotate_goal.degrees_to_rotate = msg.escape_angle
            rotate_goal.counterclockwise = True

        rotate_client.send_goal(rotate_goal)
        rotate_client.wait_for_result()

        rospy.loginfo('[Result] State: %d'%(rotate_client.get_state()))
        rospy.loginfo('[Result] Status: %s'%(rotate_client.get_goal_status_text()))

        move_forward_client.send_goal(move_forward_goal)
        move_forward_client.wait_for_result()

        rospy.loginfo('[Result] State: %d'%(move_forward_client.get_state()))
        rospy.loginfo('[Result] Status: %s'%(move_forward_client.get_goal_status_text()))

    else:
        rotate_goal.degrees_to_rotate = 180
        rotate_goal.counterclockwise = True

        rotate_client.send_goal(rotate_goal)
        rotate_client.wait_for_result()

        rospy.loginfo('[Result] State: %d'%(rotate_client.get_state()))
        rospy.loginfo('[Result] Status: %s'%(rotate_client.get_goal_status_text()))

    time.sleep(5.0)

sub = rospy.Subscriber('escape_info', Escape_info, callback, queue_size=1)
rotate_client = actionlib.SimpleActionClient('rotate', RotateAction)
move_forward_client = actionlib.SimpleActionClient('move_forward', MoveForwardAction)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

rospy.spin()