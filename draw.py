from robot_api import Robot_Api
import math
from read_gcode import read_gcode

if __name__ == "__main__":
    robot = Robot_Api("my_gen3_lite")
    robot.get_pose()

    robot.clear_faults()
    robot.example_subscribe_to_a_robot_notification()
    robot.example_set_cartesian_reference_frame()

    z = 0.024
    points = read_gcode("gcode.nc", 0.03, 0.024, 0.2, -0.2, 1)
    # robot.go_to_pose(0.2, -0.2, 0.1, 90, 0, 100, True)
    print points
    for points in points:
        robot.go_to_pose(points[0], points[1], points[2])
