#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###
from __future__ import print_function

import sys
import rospy
import time
import math
from kortex_driver.srv import *
from kortex_driver.msg import *

import actionlib


class Robot_Api:
    def __init__(self, robot_name):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', robot_name)
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(
                self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(
                self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification,
                                                     self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name,
                                                                    SetCartesianReferenceFrame)

            play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_full_name)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name,
                                                                PlayCartesianTrajectory)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(
                activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name,
                                                                GetProductConfiguration)

            get_measured_gripper_movement_full_name = '/' + self.robot_name + '/base/get_measured_gripper_movement'
            rospy.wait_for_service(get_measured_gripper_movement_full_name)
            self.get_measured_gripper_movement = rospy.ServiceProxy(get_measured_gripper_movement_full_name,
                                                                    GetMeasuredGripperMovement)



        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
            self.x6 = 0
            self.y6 = 0
            self.dx = 0
            self.dy = 0

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.005)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_joint_angles(self):
        self.last_action_notif_type = None
        # Create the list of angles
        req = PlayJointTrajectoryRequest()
        # Here the arm is vertical (all zeros)
        angles = [305.1813, 289.8717, 123.1235, 270, 103.2518, 270]
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = angles[i]
            req.input.joint_angles.joint_angles.append(temp_angle)

        # Send the angles
        rospy.loginfo("Sending the robot vertical...")

        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_gripper_command(self):
        # Initialize the request
        # Close the gripper
        gripper_command = GripperCommand()
        finger = Finger()
        position = 0.0
        finger.finger_identifier = 1
        finger.value = position

        gripper_command.gripper.finger.append(finger)
        gripper_command.mode = GripperMode.GRIPPER_POSITION

        self.send_gripper_command(gripper_command)

        time.sleep(2)
        finger.value = 0.1
        self.send_gripper_command(gripper_command)
        gripper_request = GripperRequest()

        gripper_command.mode = GripperMode.GRIPPER_SPEED
        finger.value = -0.1
        self.send_gripper_command(gripper_command)
        # GetMeasuredGripperMovementResponse
        # Wait for reported speed to be 0
        i = 0
        gripper_request.mode = GripperMode.GRIPPER_SPEED
        while True:
            i += 1
            gripper_measure = self.get_measured_gripper_movement(gripper_request)
            # print(gripper_measure.output)
            # print(dir(gripper_measure.output))
            if len(gripper_measure.output.finger):
                print("Current speed is : {0}".format(gripper_measure.output.finger[0].value))
                if gripper_measure.output.finger[0].value < 0.01:
                    if i > 5:
                        break
            else:  # Else, no finger present in answer, end loop
                break
            time.sleep(0.1)

    def gripper_open(self):
        gripper_command = GripperCommand()
        finger = Finger()
        position = 0.0
        finger.finger_identifier = 1
        finger.value = position
        gripper_command.gripper.finger.append(finger)
        gripper_command.mode = GripperMode.GRIPPER_POSITION
        self.send_gripper_command(gripper_command)
        time.sleep(2)

    def get_pose(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        print("x %f" % feedback.base.commanded_tool_pose_x)
        print("y %f" % feedback.base.commanded_tool_pose_y)
        print("z %f" % feedback.base.commanded_tool_pose_z)

        print("theta x %f" % feedback.base.commanded_tool_pose_theta_x)
        print("theta y %f" % feedback.base.commanded_tool_pose_theta_y)
        print("theta z %f" % feedback.base.commanded_tool_pose_theta_z)

    def go_to_pose(self, x, y, z, theta_x=0.0, theta_y=0.0, theta_z=0.0, theta=False, speed=0.1):
        self.last_action_notif_type = None
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()

        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        if theta:
            req.input.target_pose.theta_x = theta_x
            req.input.target_pose.theta_y = theta_y
            req.input.target_pose.theta_z = theta_z

        req.input.target_pose.x = x
        req.input.target_pose.y = y
        req.input.target_pose.z = z

        pose_speed = CartesianSpeed()
        pose_speed.translation = speed
        pose_speed.orientation = 15

        req.input.constraint.oneof_type.speed.append(pose_speed)

        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def init_start(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.x6 = feedback.base.commanded_tool_pose_x
        self.y6 = feedback.base.commanded_tool_pose_y
        self.z_free = 0.035

    def draw_circle(self):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        import numpy as np
        r, a, b = 2, 20, -20
        theta = np.arange(0, 2 * np.pi, 0.2)
        x = a + r * np.cos(theta)
        y = b + r * np.sin(theta)
        z = 0.0305
        circle = [[x[i] / 100, y[i] / 100, z] for i in range(len(x))]
        # for this_target in circle:
        # success &= self.example_send_cartesian_pose(target_position=this_target)

        for this_target in circle:
            target_x = this_target[0]
            target_y = this_target[1]
            target_z = this_target[2]
            print(target_x)
            print(target_y)
            req = PlayCartesianTrajectoryRequest()
            req.input.target_pose.x = target_x
            req.input.target_pose.y = target_y
            req.input.target_pose.z = target_z
            req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
            req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
            req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z
            # req.input.target_pose.x = feedback.base.commanded_tool_pose_x
            # req.input.target_pose.y = feedback.base.commanded_tool_pose_y
            # req.input.target_pose.z = feedback.base.commanded_tool_pose_z + 0.10
            # req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
            # req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
            # req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

            pose_speed = CartesianSpeed()
            pose_speed.translation = 0.15  # 0.1
            pose_speed.orientation = 15  # 15
            req.input.constraint.oneof_type.speed.append(pose_speed)
            rospy.loginfo("Sending the robot to the cartesian pose...")
            self.play_cartesian_trajectory(req)
            self.wait_for_action_end_or_abort()

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        return cartesianWaypoint

    def cartesian_waypoint_action(self, waypoints):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient(
            '/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory',
            kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        config = self.get_product_configuration()
        print(config.output.model)

        if config.output.model == ModelId.MODEL_ID_L31:

            for i in range(len(waypoints)):
                waypoint = waypoints[i]
                if i == 0 or i == len(waypoints) - 1:

                    goal.trajectory.append(
                        self.FillCartesianWaypoint(waypoint[0], waypoint[1], waypoint[2], math.radians(waypoint[3]),
                                                   math.radians(waypoint[4]),
                                                   math.radians(waypoint[5]), 0))
                else:
                    goal.trajectory.append(
                        self.FillCartesianWaypoint(waypoint[0], waypoint[1], waypoint[2], math.radians(waypoint[3]),
                                                   math.radians(waypoint[4]),
                                                   math.radians(waypoint[5]), 0))
            # print(goal.trajectory)
        # Call the service
        rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
        try:
            client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            client.wait_for_result()
            return True
