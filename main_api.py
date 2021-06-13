#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the
# terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import threading
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from sequence import check_for_end_or_abort

from read_gcode import read_gcode

TIMEOUT_DURATION = 20


class GripperCommandExample:
    def __init__(self, router, proportional_gain=2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(router)

    def go_to_pose(self, x, y, z, theta_x=0.0, theta_y=0.0, theta_z=0.0, theta=False):
        base = self.base
        base_cyclic = self.base_cyclic
        constrained_pose = Base_pb2.ConstrainedPose()
        feedback = base_cyclic.RefreshFeedback()

        cartesian_pose = constrained_pose.target_pose
        cartesian_pose.x = x  # (meters)
        cartesian_pose.y = y  # (meters)
        cartesian_pose.z = z  # (meters)
        cartesian_pose.theta_x = feedback.base.tool_pose_theta_x  # (degrees)
        cartesian_pose.theta_y = feedback.base.tool_pose_theta_y  # (degrees)
        cartesian_pose.theta_z = feedback.base.tool_pose_theta_z  # (degrees)
        if theta:
            cartesian_pose.theta_x = theta_x  # (degrees)
            cartesian_pose.theta_y = theta_y  # (degrees)
            cartesian_pose.theta_z = theta_z  # (degrees)

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Reaching cartesian pose...")
        base.PlayCartesianTrajectory(constrained_pose)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def gripper_open(self):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.00
        finger.finger_identifier = 1
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        self.base.SendGripperCommand(gripper_command)
        time.sleep(2)


    def ExampleSendGripperCommands(self):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.00
        finger.finger_identifier = 1
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        self.base.SendGripperCommand(gripper_command)
        # while position < 1.0:
        #     finger.value = position
        #     print("Going to position {:0.2f}...".format(finger.value))
        #     self.base.SendGripperCommand(gripper_command)
        #     position += 0.1
        time.sleep(2)

        # Set speed to open gripper
        print ("Opening gripper using speed command...")
        # gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        # gripper_request.mode = Base_pb2.GRIPPER_POSITION
        # while True:
        #     gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
        #     if len(gripper_measure.finger):
        #         print("Current position is : {0}".format(gripper_measure.finger[0].value))
        #         if gripper_measure.finger[0].value < 0.01:
        #             break
        #     else:  # Else, no finger present in answer, end loop
        #         break

        # Set speed to close gripper
        print ("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = -0.1
        self.base.SendGripperCommand(gripper_command)

        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_SPEED
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.02:
                    break
            else:  # Else, no finger present in answer, end loop
                break


def main():
    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        example = GripperCommandExample(router)
        example.go_to_pose(0.4, -0.2, 0.05, 90, 0, 100, True)
        example.ExampleSendGripperCommands()
        a = input("next")
        example.go_to_pose(0.4, -0.2, 0.2, 90, 0, 100, True)
        example.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True)
        example.go_to_pose(0.2, -0.2, 0.03, 90, 0, 100, True)

        points = read_gcode("gcode.nc", 0.03, 0.02, 0.2, -0.2, 1)
        # print(points)
        for points in points:
            example.go_to_pose(points[0], points[1], points[2], 90, 0, 100, True)
        example.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True)
        example.go_to_pose(0.4, -0.2, 0.2, 90, 0, 100, True)
        example.go_to_pose(0.4, -0.2, 0.05, 90, 0, 100, True)
        example.gripper_open()
        example.go_to_pose(0.4, -0.2, 0.2, 90, 0, 100, True)


if __name__ == "__main__":
    main()
