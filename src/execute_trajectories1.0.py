#! /usr/bin/env python3

import sys
import copy
import rospy
import numpy
import moveit_commander
import controller_manager
from controller_manager_msgs.srv import LoadController
import moveit_msgs.msg
from geometry_msgs.msg import Vector3, Point, Quaternion
from sensor_msgs.msg import Joy
from control_msgs.msg import JointTrajectoryControllerState

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

current_value = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
actual_state = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

def spacenav_callback(data):
    global current_value
    current_value = data.axes  # Beispielhaft wird der Wert der ersten Achse abgerufen
    # print(data.axes[0])

def state_callback(data):
    global actual_state
    actual_state = data.actual.positions  # Beispielhaft wird der Wert der ersten Achse abgerufen
    # print(actual_state[0])

rospy.init_node('trajectory_controller')

# Erstellen Sie eine Instanz der JointTrajectory-Nachricht
trajectory = JointTrajectory()

data = rospy.Subscriber('/spacenav/joy', Joy, spacenav_callback)

# Setzen Sie den Namen des Controllers
trajectory.header.frame_id = 'base_link'  # Frame-ID des Roboters oder der Basis

# Setzen Sie die Gelenknamen
trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']  # Liste der Gelenknamen

# Erstellen Sie eine Instanz der JointTrajectoryPoint-Nachricht für einen Punkt in der Trajektorie
point = JointTrajectoryPoint()

# Setzen Sie die Gelenkpositionen für den Punkt
point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Liste der Gelenkpositionen

# Setzen Sie die Geschwindigkeiten für den Punkt (optional)
# point.velocities = [current_value[0], current_value[1], current_value[2], current_value[3], current_value[4], current_value[5]]  # Liste der Gelenkgeschwindigkeiten

# Setzen Sie die Dauer für den Punkt (optional)
point.time_from_start = rospy.Duration(1.0)  # Dauer seit dem Start der Trajektorie

# Fügen Sie den Punkt zur Trajektorie hinzu
trajectory.points.append(point)

# Veröffentlichen Sie die Trajektorie auf dem entsprechenden ROS-Topic
trajectory_publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

rate = rospy.Rate(10)  # Veröffentlichungsrate

while not rospy.is_shutdown():
    spacenav = rospy.Subscriber('/spacenav/joy', Joy, spacenav_callback)
    state = rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, state_callback)
    point.velocities = [current_value[0], current_value[1], current_value[2], current_value[3], current_value[4], current_value[5]]  # Liste der Gelenkgeschwindigkeiten
    point.positions = [actual_state[0], actual_state[1], actual_state[2], actual_state[3], actual_state[4], actual_state[5]]
    trajectory_publisher.publish(trajectory)
    rate.sleep()









# list_controllers = ControllerLister('/controller_manager') 

# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()    
# group = moveit_commander.MoveGroupCommander("manipulator")
# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

# group.allow_replanning(True)

#rospy.init_node('joy_subs')

# Erstellen des Subscriber-Objekts
# data = rospy.Subscriber('/spacenav/offset', Vector3, some_callback)

# print("Aktueller Wert: ", current_value)
#threeDMouse_value = rospy.Publisher('spacenav/offset', float)

# print(group.get_current_pose())

# pose: 
#   position: 
#     x: 0.8171608922814401
#     y: 0.19182737481752896
#     z: -0.005555774143171135
#   orientation: 
#     x: 0.7069406647723555
#     y: 0.707272858485055
#     z: 8.416231563487837e-06
#     w: 8.412281598883994e-06

# group_pos_values = group.get_current_pose().pose.position
# group_ori_values = group.get_current_pose().pose.orientation

# group_pos_values.x = group_pos_values.x 
# group_pos_values.y = group_pos_values.y
# group_pos_values.z = group_pos_values.z - 0.000005

# group_ori_values.x = group_ori_values.x
# group_ori_values.y = group_ori_values.y
# group_ori_values.z = group_ori_values.z
# group_ori_values.w = group_ori_values.w

# group.set_position_target([group_pos_values.x,group_pos_values.y,group_pos_values.z])
# group.set_orientation_target([group_ori_values.x,group_ori_values.y,group_ori_values.z,group_ori_values.w])

# group_variable_values = group.get_current_joint_values()

# group_variable_values[0] = numpy.deg2rad(0)
# group_variable_values[1] = 0
# group_variable_values[2] = 0
# group_variable_values[3] = 0
# group_variable_values[4] = 0
# group_variable_values[5] = 0
# group.set_joint_value_target(group_variable_values)

# plan2 = group.plan()
# group.go(wait=True)

# rospy.sleep(5)

#moveit_commander.roscpp_shutdown()
