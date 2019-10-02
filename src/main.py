#! /usr/bin/env python
import rospy
import math
import actionlib
from mazerunner.msg import Escape_info
from mazerunner.msg import MoveAction, MoveGoal, MoveResult, MoveFeedback
from std_msgs.msg import Bool


rospy.init_node('Main')

def callback(msg):
    #subscibe to the pulished topic and do FakeNLP service to get the angle that need to be cahnged     

    #do server action
    #successful_pub.publish(True)

    rospy.loginfo("Waiting for server...")
    client.wait_for_server()
    goal = MoveGoal()
    goal.distance_to_move = msg.escape_distance
    if(msg.escape_angle >= 270):
        goal.radian_to_rotate = (360 - msg.escape_angle) * 2 * math.pi / 360.0
        goal.counterclockwise = False
    else:
        goal.radian_to_rotate = msg.escape_angle * 2 * math.pi / 360.0
        goal.counterclockwise = True
    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()
    rospy.loginfo('[Result] State: %d'%(client.get_state()))
    rospy.loginfo('[Result] Status: %s'%(client.get_goal_status_text()))
    rospy.loginfo('[Result] Updates sent: %d'%(client.get_result().updates_sent))
    #successful_pub.publish(False)

def feedback_cb(feedback):
    rospy.loginfo('[Feerosdback] radian_rotated: ' + str(feedback.radian_rotated))
    rospy.loginfo('[Feerosdback] radian_left: ' + str(feedback.radian_left))
    rospy.loginfo('[Feerosdback] distance_moved: ' + str(feedback.distance_moved))
    rospy.loginfo('[Feerosdback] distance_left: ' + str(feedback.distance_left))
    rospy.loginfo('')


client = actionlib.SimpleActionClient('move_rotate',MoveAction)
sub = rospy.Subscriber('escape_info', Escape_info, callback)

#successful_pub = rospy.Publisher('needStop', Bool, queue_size=1)
rospy.spin()