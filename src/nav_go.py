#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


def movebase_client(x_goal , y_goal , theta_goal) :
    client = actionlib.SimpleActionClient('move_base' , MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Pose(x,y,z, rot_x, rot_y, rot_z, w )
    # Goal Position (x,y,z)
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.position.z = 0.0

    # Quaternion to Euler angles
    row = 0.0
    pitch = 0.0
    yaw = math.radians(theta_goal) # degree
    
    rot_q = goal.target_pose.pose.orientation
    (rot_q.x , rot_q.y , rot_q.z , rot_q.w) = quaternion_from_euler(row,pitch,yaw)

    # Goal Orientation
    goal.target_pose.pose.orientation.x = rot_q.x
    goal.target_pose.pose.orientation.y = rot_q.y
    goal.target_pose.pose.orientation.z = rot_q.z
    goal.target_pose.pose.orientation.w = rot_q.w
    
    # Send a goal to Movebase Server
    client.send_goal(goal)

    wait = client.wait_for_result()

    if not wait:
        # NG
        rospy.logerr("Action server no available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Success
        return client.get_result()

if __name__ == '__main__' :
    # Initialize a ros node
    rospy.init_node('movebase_client_py')
    result = movebase_client(0.0 , 12.0 , 180.0)

    rospy.sleep(5)

    result = movebase_client(-3.0 , 12.0 , 270.0)

    rospy.sleep(5)

    result = movebase_client(-3.0 , 6.0 , 0.0)

    rospy.sleep(5)

    result = movebase_client(0.0 , 6.0 , 90.0)