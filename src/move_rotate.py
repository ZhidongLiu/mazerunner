#! /usr/bin/env python
import rospy
import time
import actionlib
from geometry_msgs.msg import Twist, Pose
from mazerunner.msg import MoveAction, MoveGoal, MoveResult, MoveFeedback
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

current_pose = Pose()
theta = 0.0 
def callback(msg): 
    global current_pose
    current_pose = msg.pose.pose

def get_radian(current_pose):
    (roll, pitch, theta_radian) = euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
    return theta_radian


#in this method we will use euler_from_quaternion function to get the radiance
def get_target_radiance(start_radiance, radiance_to_change, isRotateCounterclock):
    #get the target radian
    target_angle_radian = 0.0
    if(isRotateCounterclock):
        target_angle_radian = (radiance_to_change + start_radiance) % (2 * math.pi)
    else:
        target_angle_radian = (-1 * radiance_to_change + start_radiance) % (2 * math.pi)
    if(target_angle_radian > math.pi):
        target_angle_radian = target_angle_radian - 2 *math.pi 
    return target_angle_radian


def do_action(goal):
    global current_pose

    start_radiance = get_radian(current_pose)

    radiance_to_change = goal.radiance_to_rotate
    distance_to_move = goal.distance_to_move
    isRotateCounterclock = goal.counterclockwise

    feedback = MoveFeedback()
    feedback.radiance_rotated = 0
    feedback.radiance_left = radiance_to_change
    feedback.distance_moved = 0
    feedback.distance_left = distance_to_move

    speed = Twist()
    speed.linear.x = 0.0
    if isRotateCounterclock:
        speed.angular.z = 0.1
    else:
        speed.angular.z = -0.1

    update_count = 0
    
    pub.publish(speed)

    target_angle_radian = get_target_radiance(start_radiance, radiance_to_change, isRotateCounterclock)
    
    while abs(get_radian(current_pose) - target_angle_radian) > 0.1:
        if server.is_preempt_requested():
            result = MoveResult()
            result.updates_sent = update_count
            server.set_preempted(result, "Move preempted")

        
        feedback.radiance_rotated = abs(start_radiance - get_radian(current_pose))
        feedback.radiance_left = abs(get_radian(current_pose) - target_angle_radian)
        print "current" + str(get_radian(current_pose))
        print "target" + str(target_angle_radian)
        print ""

        server.publish_feedback(feedback)
        update_count += 1
        time.sleep(1.0)
    
    speed.angular.z = 0.0
    speed.linear.x = 0.15
    print 'stop rotate' + str(speed.angular.z)
    pub.publish(speed)
    start_time = time.time()

    while abs((time.time() - start_time) * speed.linear.x - distance_to_move) > 0.1:
        if server.is_preempt_requested():
            result = MoveResult()
            result.updates_sent = update_count
            server.set_preempted(result, "Move preempted")

        
        feedback.distance_moved = (time.time() - start_time) * speed.linear.x
        feedback.distance_left = abs((time.time() - start_time) * speed.linear.x - distance_to_move)

        server.publish_feedback(feedback)
        update_count += 1
        time.sleep(1.0)


    speed.linear.x = 0.0
    print 'stop move' + str(speed.angular.z)
    pub.publish(speed)

    result = result = MoveResult()
    result.updates_sent = update_count
    server.set_succeeded(result, "Move completed successfully")

    

rospy.init_node('moveAndRotate')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
server = actionlib.SimpleActionServer('move_rotate', MoveAction, do_action, False)
odom_sub = rospy.Subscriber('odom', Odometry, callback)
server.start()
rospy.spin()