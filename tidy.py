#! /usr/bin/env python3

# Author: Nuha Nishat
# Date: 1/30/20

# Edited by Akshaya Agrawal
# Date: 05/25/21
'''
    The Door is placed at (-0.24, 0, 0) position. Initially the Handle is located at
    (-0.24,-0.36, 0) -> (x,y,z).
    The path of the door is such that it follows a circle centered at (0,-0.5).
    The Height of the handle from the base is 0.35
'''

import rospy
import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import argparse
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, \
    AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
import numpy as np
import math
import copy
from tf.transformations import quaternion_from_euler

# move_group_python_interface_tutorial was used as reference
DL = 0.6
RPY = 0.35
RPX = 0.17
# 0.15


class MoveRobot():

    def __init__(self):
        # Initialize moveit commander and ros node for moveit

        # To read from redirected ROS Topic
        joint_state_topic = ['joint_states:=/j2s7s300_driver/out/joint_state']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('move-kinova', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.01)
        # Set the precision of the robot
        self.client = actionlib.SimpleActionClient(
            '/j2s7s300_driver/pose_action/tool_pose',
            kinova_msgs.msg.ArmPoseAction)
        self.client.wait_for_server()
        self.fin_client = actionlib.SimpleActionClient(
            '/j2s7s300_driver/fingers_action/finger_positions',
            kinova_msgs.msg.SetFingersPositionAction)
        self.fin_client.wait_for_server()
        rospy.set_param(
            '/move_group/trajectory_execution/allowed_start_tolerance', 0.3)

        rospy.wait_for_service("/apply_planning_scene", 10.0)
        rospy.wait_for_service("/get_planning_scene", 10.0)
        rospy.wait_for_service('/j2s7s300_driver/in/home_arm')
        self.apply_scene = rospy.ServiceProxy('/apply_planning_scene',
                                              ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy('/get_planning_scene',
                                            GetPlanningScene)
        rospy.sleep(2)

        # To see the trajectory
        self.disp = moveit_msgs.msg.DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)
        rospy.Subscriber('Command', String, self.callback)
        self.pub = rospy.Publisher('open_door/message', String, queue_size=1)
        rospy.spin()

    def callback(self, data):
        

    def gripper_client(self, finger_positions):
        """Send a gripper goal to the action server."""
        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions[0])
        goal.fingers.finger2 = float(finger_positions[1])
        goal.fingers.finger3 = float(finger_positions[2])
        self.fin_client.send_goal(goal)
        if self.fin_client.wait_for_result(rospy.Duration(50.0)):
            return self.fin_client.get_result()
        else:
            self.fin_client.cancel_all_goals()
            rospy.logwarn('        the gripper action timed-out')
            return None

    def cartesian_pose_client(self, position, orientation, my_frame_id):
        """Send a cartesian goal to the action server."""
        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=my_frame_id)
        goal.pose.pose.position = geometry_msgs.msg.Point(x=position[0],
                                                          y=position[1],
                                                          z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0],
            y=orientation[1],
            z=orientation[2],
            w=orientation[3])

        print('goal.pose in client 1: {}'.format(goal.pose.pose))  # debug

        self.client.send_goal(goal)

        if self.client.wait_for_result(rospy.Duration(150.0)):
            return self.client.get_result()
        else:
            self.client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None

    def homeRobot(self):
        try:
            home = rospy.ServiceProxy('/j2s7s300_driver/in/home_arm', HomeArm)
            home()
            return None
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            home()

    def clear_all_object(self):
        self.scene.remove_world_object('ground')
        self.scene.remove_world_object('top_obstacle')
        self.scene.remove_world_object('bot_obstacle')
        self.scene.remove_world_object('table')

    def clear_table(self):
        self.scene.remove_world_object('table')

    def add_table(self, heigh=-100):
        self.scene.remove_world_object('table')
        table_size = [1, 0.6, 0.001]
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'root'
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = heigh
        self.scene.add_box('table', table_pose, table_size)

    def add_object(self):
        self.scene.remove_world_object('ground')
        self.scene.remove_world_object('top_obstacle')
        self.scene.remove_world_object('bot_obstacle')

        # 设置table的三维尺寸
        ground_size = [3, 3, 0.001]
        top_obstacle_size = [0.12, 0.15, 2]
        bot_obstacle_size = [0.45, 0.45, 0.41]

        # 将table加入场景当中
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = 'root'
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = -0.4

        top_obstacle_pose = PoseStamped()
        top_obstacle_pose.header.frame_id = 'root'
        top_obstacle_pose.pose.position.x = 0
        top_obstacle_pose.pose.position.y = 0.24
        top_obstacle_pose.pose.position.z = 0.92

        bot_obstacle_pose = PoseStamped()
        bot_obstacle_pose.header.frame_id = 'root'
        bot_obstacle_pose.pose.position.x = 0
        bot_obstacle_pose.pose.position.y = 0.12
        bot_obstacle_pose.pose.position.z = -0.41 / 2 - 0.03

        self.scene.add_box('ground', ground_pose, ground_size)
        self.scene.add_box('top_obstacle', top_obstacle_pose,
                           top_obstacle_size)
        self.scene.add_box('bot_obstacle', bot_obstacle_pose,
                           bot_obstacle_size)

        rospy.sleep(0.5)

    def set_planner_type(self, planner_name):
        if planner_name == "RRT":
            self.move_group.set_planner_id("RRTConnectkConfigDefault")
        if planner_name == "RRT*":
            self.move_group.set_planner_id("RRTstarkConfigDefault")
        if planner_name == "PRM*":
            self.move_group.set_planner_id("PRMstarkConfigDefault")

    def go_to_joint_state(self, joint_state):
        joint_goal = JointState()
        joint_goal.position = joint_state
        self.move_group.set_joint_value_target(joint_goal.position)

        self.plan = self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.execute(self.plan, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def go_to_goal(self, ee_pose):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = ee_pose[0]
        pose_goal.position.y = ee_pose[1]
        pose_goal.position.z = ee_pose[2]

        if len(ee_pose) == 6:
            quat = tf.transformations.quaternion_from_euler(
                math.radians(ee_pose[3]), math.radians(ee_pose[4]),
                math.radians(ee_pose[5]))
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]

        else:
            pose_goal.orientation.x = ee_pose[3]
            pose_goal.orientation.y = ee_pose[4]
            pose_goal.orientation.z = ee_pose[5]
            pose_goal.orientation.w = ee_pose[6]

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_planning_time(20)
        rospy.sleep(2)
        self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def move_gripper(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            self.move_gripper.set_joint_value_target(cmd)
        self.move_gripper.go(wait=True)
        rospy.sleep(2)

    # def display_trajectory(self):
    #     self.disp_pub = rospy.Publisher("/move_group/display_planned_path",
    #                                     moveit_msgs.msg.DisplayTrajectory,
    #                                     queue_size=20)
    #     self.disp.trajectory.append(self.plan)
    #     print(self.disp.trajectory)
    #     self.disp_pub.publish(self.disp)

    def go_to_finger_state(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "CurlClose":
            self.move_gripper.set_named_target("CurlClose")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            """gripper_goal = JointState()
            gripper_goal.position = cmd"""
            self.move_gripper.set_joint_value_target(cmd)
        # self.plan_gripper = self.move_gripper.plan()

        self.move_gripper.go(wait=True)
        """self.move_gripper.execute(self.plan_gripper, wait=True)"""
        # self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        rospy.sleep(1)

    def go_to_finger_joint_state(self, joint_values):
        try:
            gripper_states = JointState()
            gripper_states.position = joint_values
            self.move_gripper.set_joint_value_target(gripper_states.position)
            self.move_gripper.go(wait=True)
            # self.move_gripper.execute(self.plan_gripper, wait=True)
            self.move_gripper.stop()
            self.move_gripper.clear_pose_targets()
            rospy.sleep(0.1)
            return True
        except:
            return False

    def go_to_arm_joint_state(self, joint_values):
        try:
            arm_states = JointState()
            arm_states.position = joint_values
            # try:
            self.move_group.set_joint_value_target(arm_states.position)
            # except Exception as e:
            #     rospy.loginfo(e)
            self.move_group.go(wait=True)
            """self.move_gripper.execute(self.plan_gripper, wait=True)"""
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def main(self):
        self.add_object()
        # Set up path here

        # Pick planner
        self.set_planner_type("RRT")
        self.move_group.set_named_target('Home')
        self.move_group.go(wait=True)

        ################ Pre-grasp ###################

        # Open the Gripper
        rospy.loginfo('opening the gripper')
        self.go_to_finger_state('Open')

        rospy.loginfo('PRE-Grasp')
        self.go_to_goal(target)

        rospy.sleep(1)
        ################# Grasp ###################
        rospy.loginfo('Grasp')
        cpose = self.move_group.get_current_pose().pose
        for i in range(0, 15):
            cpose.position.y += i * 0.01
            self.cartesian_pose_client(
                [cpose.position.x, cpose.position.y, cpose.position.z], [
                    cpose.orientation.x, cpose.orientation.y,
                    cpose.orientation.z, cpose.orientation.w
                ], "/j2s7s300_link_base")
        rospy.sleep(2)
        ###### Get Waypoints ###########

        ###### Plan path ##########

        # waypoints = []
        # # rospy.loginfo('current pose: ')
        # wpose = self.move_group.get_current_pose().pose
        # wx = wpose.position.x
        # for i in (0, 15):
        #     wpose.position.x += 0.01
        #     waypoints.append(copy.deepcopy(wpose))

        # rospy.loginfo('waypoints: ', waypoints)

        # fraction = 0.0  #路径规划覆盖率
        # maxtries = 100  #最大尝试规划次数
        # attempts = 0  #已经尝试规划次数

        # # 设置机器臂当前的状态作为运动初始状态
        # self.move_group.set_start_state_to_current_state()

        # while fraction < 1.0 and attempts < maxtries:
        #     (plan, fraction) = self.move_group.compute_cartesian_path(
        #         waypoints,  # waypoint poses，路点列表
        #         0.01,  # eef_step，终端步进值
        #         0.0,  # jump_threshold，跳跃阈值
        #         True)  # avoid_collisions，避障规划

        #     # 尝试次数累加
        #     attempts += 1

        #     # 打印运动规划进程
        #     if attempts % 10 == 0:
        #         rospy.loginfo("Still trying after " + str(attempts) +
        #                       " attempts...")

        # # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        # if fraction == 1.0:
        #     rospy.loginfo("Path computed successfully. Moving the arm.")
        #     self.move_group.execute(plan, wait=True)
        #     rospy.loginfo("Path execution complete.")
        # # 如果路径规划失败，则打印失败信息
        # else:
        #     rospy.loginfo("Path planning failed with only " + str(fraction) +
        #                   " success after " + str(maxtries) + " attempts.")

        self.go_to_finger_state('Close')
        rospy.sleep(1)
        rospy.loginfo("Go home.")
        self.move_group.set_named_target('Home')
        self.move_group.go(wait=True)

        # self.homeRobot()


if __name__ == '__main__':
    MoveRobot()