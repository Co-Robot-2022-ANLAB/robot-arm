#!/usr/bin/env python

# Author: Hyeonjun Park, Ph.D. candidate
# Affiliation: Human-Robot Interaction LAB, Kyung Hee University, South Korea
# koreaphj91@gmail.com
# init: 9 Apr 2019
# revision: 19 Mar 2020

import sys
import rospy
import copy, math
from math import pi
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

GROUP_NAME_ARM = "ur3_arm"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

group_name = "ur3_arm"
pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move',anonymous=True)
group = [moveit_commander.MoveGroupCommander(group_name)]  # ur3 moveit group name: manipulator


class TestMove():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur3_move',anonymous=True)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.robot_arm.set_goal_orientation_tolerance(0.0005)
        self.robot_arm.set_planning_time(10000)
        self.robot_arm.set_num_planning_attempts(100)

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)        

    # def move_up(self):
          
    #     self.robot_arm.set_named_target("up")  #go to goal state.
    #     self.robot_arm.go(wait=True)
    #     print("====== move plan go to home 1 ======")        
    #     rospy.sleep(1)

    def gripper_open(self):
        #gripper open
        self.command = 'open'
        rospy.sleep(5)

    def gripper_close(self):
        #gripper close
        self.command = 'close'
        rospy.sleep(5)
        
#     def move_down(self):    
# #        print("====== move plan go to up ======")
#         self.robot_arm.set_named_target("home")  #go to goal state.
#         self.robot_arm.go(wait=True)
#         print("====== move plan go to up ======")        
#         rospy.sleep(1)       

    def move_p1(self):    
#        print("====== move plan go to up ======")
        self.robot_arm.set_named_target("p1")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to p1 ======")        
        rospy.sleep(1)

    def move_p2(self):    
#        print("====== move plan go to up ======")
        self.robot_arm.set_named_target("p2")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to p2 ======")        
        rospy.sleep(1)       

    def move_p3(self):    
#        print("====== move plan go to up ======")
        self.robot_arm.set_named_target("p3")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to p3 ======")        
        rospy.sleep(1)       

    def state_print(self):
        robot_state = self.robot_arm.get_current_pose();
        robot_angle = self.robot_arm.get_current_joint_values();

        print(robot_state)
        #print(robot_angle)


while not rospy.is_shutdown():        
# if __name__=='__main__':
    
    tm = TestMove()

    #gripper publisher
    pub = rospy.Publisher('gripper/command', String, queue_size=10)
    rospy.Rate(2)

    tm.__init__()
    
    #voltage High
    pub.publish('highvoltage')
    rospy.sleep(2)

    #open gripper
    pub.publish('open')
    rospy.sleep(2)

    #close gripper
    pub.publish('close')
    rospy.sleep(2)

    #move up
    tm.move_p1()
    rospy.sleep(1)

    tm.move_p2()
    rospy.sleep(1)

    tm.move_p3()
    rospy.sleep(1)


    # #open gripper
    # pub.publish('open')
    # rospy.sleep(2)

    # #close gripper
    # pub.publish('close')
    # rospy.sleep(2)

    # # state display
    # tm.state_print()

    # rospy.spin()
    # roscpp_shutdown()
moveit_commander.roscpp_shutdown()