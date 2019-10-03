#! /usr/bin/env python
import rospy
import math
import time
import actionlib
from geometry_msgs.msg import Twist
from mazerunner.msg import Escape_info
from mazerunner.msg import RotateAction, RotateGoal, RotateResult, RotateFeedback
from std_msgs.msg import Bool


rospy.init_node('Main')

def callback(msg):
    #subscibe to the pulished topic and do FakeNLP service to get the angle that need to be changed     

    twist = Twist()
    pub.publish(twist)

    print msg.escape_angle
    print msg.escape_distance
    time.sleep(2.0)

    rospy.loginfo("Waiting for server...")
    client.wait_for_server()
    goal = RotateGoal()
    goal.distance = msg.escape_distance
    goal.angular_velocity = math.pi/10
    goal.linear_velocity = 0.1

    if msg.escape_angle >= 270:
        goal.degrees_to_rotate = 360 - msg.escape_angle
        goal.counterclockwise = False
    else:
        goal.degrees_to_rotate = msg.escape_angle
        goal.counterclockwise = True

    client.send_goal(goal)
    # client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()
    rospy.loginfo('[Result] State: %d'%(client.get_state()))
    rospy.loginfo('[Result] Status: %s'%(client.get_goal_status_text()))
    time.sleep(4)

sub = rospy.Subscriber('escape_info', Escape_info, callback)
client = actionlib.SimpleActionClient('rotate',RotateAction)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
time.sleep(2.0)

rospy.spin()