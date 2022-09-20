#!/usr/bin/env python

# Author: Hyeonjun Park, Ph.D. candidate
# Affiliation: Human-Robot Interaction LAB, Kyung Hee University, South Korea
# koreaphj91@gmail.com
# init: 9 Apr 2019
# revision: 19 Mar 2020

import sys
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import copy, math
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import String, Float32MultiArray, Float64, Bool, Int32
import moveit_commander
import moveit_msgs.msg

from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

global center
global angle
global safe
global hand_num
global tool_num
global Robot
global tool_point

client = None
safe = False
center = [0.0, 0.0]
centerii =[575.0, 57.0]
angle = 0.0
hand_num = 0
tool_num = 2
Robot = False
tool_point = 'NO'

GROUP_NAME_ARM = "ur3_arm"
FIXED_FRAME = 'world'
#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [0,0,0,0,0,0]

group_name = "ur3_arm"
pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move',anonymous=True)
group = [moveit_commander.MoveGroupCommander(group_name)]  # ur3 moveit group name: manipulator

# setting ValueError
def stop(safe):
    allowed = [False]
    if safe not in allowed:
        raise ValueError

class TestMove():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        rospy.init_node('ur3_move',anonymous=True)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(1000)
        self.robot_arm.set_num_planning_attempts(300)

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True)  

        rospy.sleep(5)

    def gripper_close(self):
        self.command = 'close'
        rospy.sleep(1)
        
    def move_pose1(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("pose1")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to pose1 ======")
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise        

    def move_pose2(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("pose2")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to pose2 ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_pose3(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("pose3")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to pose3 ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_home(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("home")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to home ======") 
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_grip1(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip1")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip1 ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
    def move_grip2(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip2")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip2 ======")
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise        

    def move_grip3(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip3")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip3 ======")
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_grip4(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip4")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip4 ======")
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_grip5(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip5")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip5 ======")
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    

    def move_grip6(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip6")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip6 ======")
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise   

    def move_grip7(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip7")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip7 ======")    
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_grip8(self):
        try:
            stop(safe)
            self.robot_arm.set_named_target("grip8")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to grip8 ======")   
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise    
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise 

    def move_toolbox1(self):
        try:
            self.robot_arm.set_named_target("toolbox1")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to toolbox1 ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_toolbox2(self):
        try:
            self.robot_arm.set_named_target("toolbox2")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to toolbox2 ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise        

    def move_toolbox3(self):
        try:
            self.robot_arm.set_named_target("toolbox3")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to toolbox3 ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise         

    def move_toolbox4(self):
        try:
            self.robot_arm.set_named_target("toolbox4")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to toolbox4 ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise

    def move_mid_point(self):
        try:
            self.robot_arm.set_named_target("mid_point")  #go to goal state.
            self.robot_arm.go(wait=True)
            print("====== move plan go to mid_point ======")        
        except KeyboardInterrupt:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise
        except ValueError:
            self.robot_arm.stop()
            self.move_group.clear_pose_targets()
            raise   

    def state_print(self):
        robot_state = self.robot_arm.get_current_pose()
        robot_angle = self.robot_arm.get_current_joint_values()

        print(robot_state)
        print(robot_angle)

def callback_angle(data1):
    global angle
    angle = data1.data 
    
def callback_center(data2):
    global center
    center = data2.data

def callback_safe(data3):
    global safe
    safe = data3.data

def callback_hand(data4):
    global hand_num
    hand_num = data4.data

def callback_tool(data5):
    global tool_num
    tool_num = data5.data

def callback_robot(data6):
    global Robot
    Robot = data6.data

def callback_tool_point(data7):
    global tool_point
    tool_point = data7.data

def go_angle():
    global joints_pos
    global angle

    angle_rad = (-angle)*pi/180
    Q1[5] = angle_rad

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=[x+y for x,y in  zip(joints_pos,Q1)], velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except ValueError:
        client.cancel_goal()
        raise    

while not rospy.is_shutdown():        
    global client
    tm = TestMove()

    #gripper publisher
    pub = rospy.Publisher('gripper/command', String, queue_size=10)
    flag_pub = rospy.Publisher('angle_flag', Bool, queue_size=10)

    # ros subscribe
    tool_sub1 = rospy.Subscriber("/camera/tool_angle", Float64, callback_angle, queue_size=100)
    tool_sub1 = rospy.Subscriber("/camera/tool_center", Float32MultiArray, callback_center, queue_size=100)
    safe_sub = rospy.Subscriber("/safe_zone", Bool, callback_safe, queue_size=100)
    hand_sub = rospy.Subscriber("/status_hand", Int32, callback_hand, queue_size=100)
    robot_sub = rospy.Subscriber("/robot_zone", Bool, callback_robot, queue_size=100)
    tool_point_sub = rospy.Subscriber("/camera/tool_position", String, callback_tool_point, queue_size=100)
    status_pub = rospy.Publisher('robot/status', String, queue_size=10)
    rospy.Rate(2)

    float_center = [float(i) for i in center]
    tool_center_x = (float_center[1] -250) /1000
    tool_center_y = (-875 + float_center[0])/ 1000

    tm.__init__()
    
    pub.publish('close')
    tm.move_home()

    status_pub.publish('waiting')

    # print(hand_num)
    # print(safe)

    flag_pub.publish(True)

    #voltage High
    pub.publish('highvoltage')
    rospy.sleep(1)

    # plier pick up to zone -- not yet work ( pose1, toolbox1 is not located )
    try:    
        if hand_num == 102:
            client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            
            status_pub.publish('bring plier')

            tm.move_pose1()
            pub.publish('open')
            rospy.sleep(1)
            tm.move_toolbox1()
            pub.publish('close')
            rospy.sleep(1)
            tm.move_pose1()
            tm.move_home()
            tm.move_grip4()
            pub.publish('open')
            rospy.sleep(1)
            tm.move_mid_point()
            pub.publish('close')
            rospy.sleep(1)
            tm.move_home()
            
            hand_num = 0

        # hammer pick up
        elif hand_num == 203:

            status_pub.publish('bring hammer')

            tm.move_pose2()
            pub.publish('open')
            rospy.sleep(1)
            tm.move_toolbox2()
            pub.publish('close')
            rospy.sleep(1)
            tm.move_pose2()
            tm.move_home()
            tm.move_grip4()
            pub.publish('open')
            rospy.sleep(1)
            tm.move_mid_point()
            pub.publish('close')
            rospy.sleep(1)
            tm.move_home()

            hand_num = 0

        # driver pick up to zone
        elif hand_num == 304:

            status_pub.publish('bring driver')

            tm.move_pose3()
            pub.publish('open')
            rospy.sleep(1)
            tm.move_toolbox3()
            pub.publish('close')
            rospy.sleep(1)
            tm.move_pose3()
            tm.move_home()
            tm.move_grip4()
            pub.publish('open')
            rospy.sleep(1)
            tm.move_mid_point()
            pub.publish('close')
            rospy.sleep(1)
            tm.move_home()

            hand_num = 0

        # tool repatriate to toolbox
        if hand_num == 444:

            flag_pub.publish(False)
            client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)

            status_pub.publish('bring out tool')

            if tool_point == 'P1':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip1()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()
            
            elif tool_point == 'P2':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip2()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()
        
            elif tool_point == 'P3':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip3()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            elif tool_point == 'P4':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip4()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            elif tool_point == 'P5':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip5()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            elif tool_point == 'P6':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip6()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            elif tool_point == 'P7':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip7()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            elif tool_point == 'P8':
                tm.move_mid_point()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_grip8()
                go_angle()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            # tool1(plier)
            if tool_num == 1:

                status_pub.publish('replace plier')

                tm.move_pose1()
                tm.move_toolbox1()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_pose1()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            # tool2(hammer)
            elif tool_num == 2:

                status_pub.publish('replace hammer')

                tm.move_pose2()
                tm.move_toolbox2()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_pose2()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            # tool3(driver)
            elif tool_num == 3:

                status_pub.publish('replace driver')

                tm.move_pose3()
                tm.move_toolbox3()
                pub.publish('open')
                rospy.sleep(1)
                tm.move_pose3()
                pub.publish('close')
                rospy.sleep(1)
                tm.move_home()

            flag_pub.publish(True)

            hand_num = 0

        # toolbox4(handicraft)
        elif hand_num == 111:

            status_pub.publish('replace handicraft')            

            flag_pub.publish(False)
            tm.move_mid_point()
            pub.publish('open')
            rospy.sleep(1)
            tm.move_grip1()
            go_angle()
            pub.publish('close')
            rospy.sleep(1)
            tm.move_toolbox4()
            pub.publish('open')
            rospy.sleep(1)
            pub.publish('close')
            tm.move_home()

            flag_pub.publish(True)

            hand_num = 0

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

    # rospy.spin()
    # roscpp_shutdown()

moveit_commander.roscpp_shutdown()