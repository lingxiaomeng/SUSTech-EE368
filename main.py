from robot_api import Robot_Api
import math

from writing.read_gcode import read_gcode

if __name__ == "__main__":
    robot = Robot_Api("my_gen3_lite")
    robot.get_pose()

    robot.clear_faults()
    robot.example_subscribe_to_a_robot_notification()
    robot.example_set_cartesian_reference_frame()
    points = read_gcode("gcode.nc", 0.03, 0.018, 0.2, -0.2, 0.5)
    print(points)
    # a = input()
    z = 0.1
    # robot.go_to_pose(0.2, -0.2, z, 90, 0, 100, True)
    # robot.example_cartesian_waypoint_action()

    robot.go_to_pose(0.4, -0.2, 0.05, 90, 0, 100, True)

    a = input("next")
    robot.example_send_gripper_command()
    robot.go_to_pose(0.4, -0.2, 0.2, 90, 0, 100, True)
    robot.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True)
    robot.go_to_pose(0.2, -0.2, 0.03, 90, 0, 100, True)

    # print(points)
    # waypoints = []
    for points in points:
        robot.go_to_pose(points[0], points[1], points[2], 90, 0, 100, theta=True, speed=points[3])
        # waypoints.append((points[0], points[1], points[2], 90, 0, 100))
    # robot.cartesian_waypoint_action(waypoints)
    robot.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True)
    robot.go_to_pose(0.4, -0.2, 0.2, 90, 0, 100, True)
    robot.go_to_pose(0.4, -0.2, 0.05, 90, 0, 100, True)
    robot.gripper_open()
    robot.go_to_pose(0.4, -0.2, 0.2, 90, 0, 100, True)
