#! /usr/bin/env python
import rospy
import actionlib
import time
from geometry_msgs.msg import Twist
from mazerunner.msg import MoveForwardAction, MoveForwardGoal, MoveForwardResult

# Action Request Comes in
def do_move(goal):

    relative_angle = 0
    target_distance = goal.distance
    rate = rospy.Rate(10)

    move_forward_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move_msg = Twist()

    # start_time = time.time()
    # while (time.time() - start_time) * move_msg.linear.x < target_distance:
    #     move_msg.linear.x = goal.linear_velocity
    #     move_forward_publisher.publish(move_msg)
    #     print "Moving Signal given"
    #     distance_left = target_distance - (time.time() - start_time) * move_msg.linear.x
    #     print "distance left, %s" % distance_left
    #     rate.sleep()

    start_time = time.time()
    while (time.time() - start_time) * move_msg.linear.x < target_distance * 0.25:
        move_msg.linear.x = goal.linear_velocity * 0.5
        move_forward_publisher.publish(move_msg)
        print "Moving Signal given"
        distance_left = target_distance - (time.time() - start_time) * move_msg.linear.x
        print "distance left, %s" % distance_left
        rate.sleep()
    start_time = time.time()
    while (time.time() - start_time) * move_msg.linear.x < target_distance * 0.5:
        move_msg.linear.x = goal.linear_velocity
        move_forward_publisher.publish(move_msg)
        print "Moving Signal given"
        distance_left = target_distance - (time.time() - start_time) * move_msg.linear.x
        print "distance left, %s" % distance_left
        rate.sleep()
    start_time = time.time()
    move_msg.linear.x = goal.linear_velocity * 0.5
    while (time.time() - start_time) * move_msg.linear.x < target_distance * 0.25:
        move_forward_publisher.publish(move_msg)
        print "Moving Signal given"
        distance_left = 0.25 * target_distance - (time.time() - start_time) * move_msg.linear.x
        print "distance left, %s" % distance_left
        rate.sleep()

    move_msg.linear.x = 0.0
    move_forward_publisher.publish(move_msg)
    result = MoveForwardResult()
    server.set_succeeded(result, "Move Completed")
    time.sleep(2.0)

# Declare that we are a node
rospy.init_node('move_forward')

# Declare that this node will handle actions
# When action requests come in, call do_timer method
server = actionlib.SimpleActionServer('move_forward', MoveForwardAction, do_move, False)

# Start it up
server.start()

# Wait until ^c
rospy.spin()