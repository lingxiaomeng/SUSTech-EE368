from robot_api import Robot_Api
import math

if __name__ == "__main__":
    robot = Robot_Api("my_gen3_lite")
    robot.get_pose()

    robot.clear_faults()
    robot.example_subscribe_to_a_robot_notification()
    robot.example_set_cartesian_reference_frame()

    z = 0.1
    robot.go_to_pose(0.2, -0.2, z, 90, 0, 100, True)
    robot.go_to_pose(0.3, -0.2, z, speed=0.01)
    robot.go_to_pose(0.3, -0.3, z, speed=0.05)
    robot.go_to_pose(0.2, -0.3, z, speed=0.1)
    robot.go_to_pose(0.2, -0.2, z, speed=0.3)

    # robot.go_to_pose(0.15, -0.27, z)
    # robot.go_to_pose(0.15, -0.32, z)
    # robot.go_to_pose(0.2, -0.32, z)
    # #
    # # robot.go_to_pose(0.2, -0.2, z + 0.03)
