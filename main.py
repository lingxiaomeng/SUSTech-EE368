from robot_api import Robot_Api
import math

if __name__ == "__main__":
    robot = Robot_Api("my_gen3_lite")
    robot.get_pose()

    robot.clear_faults()
    robot.example_subscribe_to_a_robot_notification()
    robot.example_set_cartesian_reference_frame()

    z = 0.1
    # robot.go_to_pose(0.2, -0.2, z, 90, 0, 100, True)
    # robot.example_cartesian_waypoint_action()
    robot.example_send_gripper_command()

    waypoints = []
    for i in range(50):
        x = 0.2 + 0.05 * math.cos(float(i) / 25 * math.pi)
        y = -0.2 + 0.05 * math.sin(float(i) / 25 * math.pi)
        waypoint = (x, y, z, 90, 0, 100)
        waypoints.append(waypoint)
        # print(waypoints)
    # robot.cartesian_waypoint_action(waypoints)
        # print("x %f y %f" % (x, y))
        # robot.go_to_pose(0.2 + x, -0.2 + y, z)
        # robot.go_to_pose(0.15, -0.27, z)
        # robot.go_to_pose(0.15, -0.32, z)
        # robot.go_to_pose(0.2, -0.32, z)
    # #
    # # robot.go_to_pose(0.2, -0.2, z + 0.03)
