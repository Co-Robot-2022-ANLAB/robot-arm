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

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"

class TestMove():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur3_move',anonymous=True)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(10)
        self.robot_arm.set_num_planning_attempts(10)

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)        

    def move_up(self):
          
        self.robot_arm.set_named_target("up")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to home 1 ======")        
        rospy.sleep(1)

    def gripper_open(self):
        #gripper open
        self.command = 'open'
        rospy.sleep(5)

    def gripper_close(self):
        #gripper close
        self.command = 'close'
        rospy.sleep(5)
        
    def move_down(self):    
#        print("====== move plan go to up ======")
        self.robot_arm.set_named_target("home")  #go to goal state.
        self.robot_arm.go(wait=True)
        print("====== move plan go to up ======")        
        rospy.sleep(1)       

#        robot_arm.set_named_target("up")
#        robot_arm.go(wait=True)

    def state_print(self):
        robot_state = self.robot_arm.get_current_pose();
        robot_angle = self.robot_arm.get_current_joint_values();

        print(robot_state)
        #print(robot_angle)

        
if __name__=='__main__':
    
    tm = TestMove()
    

    #gripper publisher
    pub = rospy.Publisher('gripper/command', String, queue_size=10)
    rospy.Rate(10)

    tm.__init__()

    #voltage High
    pub.publish('highvoltage')
    rospy.sleep(5)

    #open gripper
    pub.publish('open')
    rospy.sleep(2)

    #close gripper
    pub.publish('close')
    rospy.sleep(2)

    #move up
    tm.move_up()

    #gripper open
    pub.publish('open')
    rospy.sleep(2)
    
    #move down
    # tm.move_down()

    # #gripper close
    # pub.publish('close')
    # rospy.sleep(2)

    # #state display
    # tm.state_print()

    rospy.spin()
    roscpp_shutdown()
