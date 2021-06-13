from read_gcode import read_gcode
from robot_api import Robot_Api


def main():
    robot = Robot_Api("my_gen3_lite")
    robot.get_pose()
    robot.clear_faults()
    robot.example_subscribe_to_a_robot_notification()
    robot.example_set_cartesian_reference_frame()
    points = read_gcode("gcode.nc", 0.03, 0.006, 0.2, -0.2, 0.5)
    robot.gripper_open()
    robot.example_retract_the_robot()
    joints = robot.IK(0.3, -0.4, 0.055)
    robot.example_send_joint_angles(joints)
    joints = robot.IK(0.36, -0.48, 0.055)
    robot.example_send_joint_angles(joints)
    robot.example_send_gripper_command()
    robot.go_to_pose(0.3 * 1.2, -0.4 * 1.2, 0.2)
    robot.example_home_the_robot()
    robot.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True, 0.18)
    robot.go_to_pose(0.2, -0.2, 0.03, 90, 0, 100, True, 0.18)
    for points in points:
        robot.go_to_pose(points[0], points[1], points[2], 90, 0, 100, theta=True, speed=points[3])
    robot.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True, 0.18)
    robot.example_home_the_robot()
    robot.go_to_pose(0.36, -0.48, 0.2, 90, 0, 40, True, 0.18)
    robot.go_to_pose(0.36, -0.48, 0.06)
    robot.gripper_open()
    robot.go_to_pose(0.36, -0.48, 0.2)
    robot.example_home_the_robot()
    robot.example_retract_the_robot()


if __name__ == "__main__":
    main()
