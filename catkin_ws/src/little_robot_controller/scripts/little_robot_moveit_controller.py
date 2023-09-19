#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import math
import geometry_msgs.msg
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def move_cartesian(x, y, z):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_demo', anonymous=True)

    arm_group = moveit_commander.MoveGroupCommander('littleRobot_arm')  # 'littleRobot_arm' es el nombre del grupo de articulaciones

    # Definir la posición en coordenadas cartesianas
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    # Orientación (en este caso, es una orientación arbitraria)
    pose_target.orientation.w = 1.0

    arm_group.set_pose_target(pose_target)

    plan = arm_group.plan()
    arm_group.go(wait=True)
    arm_group.clear_pose_targets()

def move_joint(joint1, joint2, joint3):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_demo', anonymous=True)

    arm_group = moveit_commander.MoveGroupCommander('littleRobot_arm')  # 'littleRobot_arm' es el nombre del grupo de articulaciones

    joint_values = arm_group.get_current_joint_values()
    joint_values[0] = joint1  # Valor para la articulación 1
    joint_values[1] = joint2  # Valor para la articulación 2
    joint_values[2] = joint3  # Valor para la articulación 3

    arm_group.set_joint_value_target(joint_values)
    arm_group.go(wait=True)
    arm_group.clear_pose_targets()


def callback(data):
    # Aquí puedes procesar la trayectoria
    trajectory_points = data.joint_trajectory.points
    for point in trajectory_points:
        print(f"Tiempo: {point.time_from_start.to_sec()}s")
        print(f"Posiciones de las articulaciones: {point.positions}")

def main():
    rospy.init_node('trajectory_listener', anonymous=True)
    rospy.Subscriber("/move_group/display_planned_path", moveit_msgs.msg.RobotTrajectory, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        # move_cartesian(0.0, 0.0, 0.24)  # Puedes ajustar las coordenadas según lo necesites
        # main()
        move_joint(-1.57, 0, 0)  # Puedes ajustar los valores según lo necesites
    except rospy.ROSInterruptException:
        pass
