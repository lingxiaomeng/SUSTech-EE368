# coding=utf-8
from read_gcode import read_gcode
from robot_api import Robot_Api


def main():
    robot = Robot_Api("my_gen3_lite")
    robot.get_pose()
    robot.clear_faults()
    robot.example_subscribe_to_a_robot_notification()
    robot.example_set_cartesian_reference_frame()

    # 读取gcode生成轨迹
    points = read_gcode("gcode_sim.nc", 0.03, 0.006, 0.2, -0.2, 0.5)
    robot.gripper_open()  # 张开机械爪
    robot.example_retract_the_robot()  # 回到Retract位置
    joints = robot.IK(0.3, -0.4, 0.055)
    robot.example_send_joint_angles(joints)  # 运动到抓笔姿态
    joints = robot.IK(0.36, -0.48, 0.055)
    robot.example_send_joint_angles(joints)  # 前往笔的位置
    robot.example_send_gripper_command()  # 机械爪抓取
    robot.go_to_pose(0.3 * 1.2, -0.4 * 1.2, 0.2)  # 机械臂抬起
    robot.example_home_the_robot()  # 前往工作位置
    robot.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True, 0.18)
    robot.go_to_pose(0.2, -0.2, 0.03, 90, 0, 100, True, 0.18)  # 前往写字位置
    for points in points:  # 写字
        robot.go_to_pose(points[0], points[1], points[2], 90, 0, 100, theta=True, speed=points[3])
    robot.go_to_pose(0.2, -0.2, 0.2, 90, 0, 100, True, 0.18)  # 抬笔
    robot.example_home_the_robot()  # 前往工作位置
    robot.go_to_pose(0.36, -0.48, 0.2, 90, 0, 40, True, 0.18)  # 前往笔筒上方
    robot.go_to_pose(0.36, -0.48, 0.06)  # 放下
    robot.gripper_open()  # 机械爪松开
    robot.go_to_pose(0.36, -0.48, 0.2)  # 机械臂抬起
    robot.example_home_the_robot()  # 前往工作位置
    robot.example_retract_the_robot()  # 回到初始位置


if __name__ == "__main__":
    main()
