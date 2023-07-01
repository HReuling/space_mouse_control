#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Int16

current_value = (0,0,0,0,0,0)
buttons = (0,0)
current_gripper_position = 0

def spacenav_callback(data):
    global current_value
    current_value = data.axes  # Beispielhaft wird der Wert der ersten Achse abgerufen
    global buttons 
    buttons = data.buttons

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("manipulator") # real: ur3_arm, gazebo: manipulator
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
gripper_position_publisher = rospy.Publisher('/robotiq_gripper/position', Int16, queue_size=10)

group.allow_replanning(True)
group.set_pose_reference_frame("world") # reference coordinate system
group.set_planning_time(1)
group.set_end_effector_link("tool0") # set TCP

speed_linear = 0.05
speed_angular = -0.2
gripper_speed = 10

rate = rospy.Rate(10)  # Veröffentlichungsrate
while not rospy.is_shutdown():
    spacenav = rospy.Subscriber('/spacenav/joy', Joy, spacenav_callback)
    waypoints = []
    new_pose = group.get_current_pose()
    new_rpy = group.get_current_rpy()

    new_pose.pose.position.x = new_pose.pose.position.x + current_value[0]*speed_linear
    new_pose.pose.position.y = new_pose.pose.position.y + current_value[1]*speed_linear
    new_pose.pose.position.z = new_pose.pose.position.z + current_value[2]*speed_linear

    rot_qua = quaternion_from_euler(current_value[5]*speed_angular,current_value[4]*speed_angular,current_value[3]*speed_angular)

    pose_qua =  [new_pose.pose.orientation.x, new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w]

    new_qua = quaternion_multiply(pose_qua, rot_qua)

    new_pose.pose.orientation.x = new_qua[0]
    new_pose.pose.orientation.y = new_qua[1]
    new_pose.pose.orientation.z = new_qua[2]
    new_pose.pose.orientation.w = new_qua[3]

    waypoints.append(copy.deepcopy(new_pose.pose))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.05, 0.0)
    group.execute(plan, wait=False)
    waypoints.clear()
    
    # calculate new gripper position
    current_gripper_position = current_gripper_position + (buttons[0] - buttons[1])*gripper_speed
    # set the value between 0 and 255
    if current_gripper_position < 0: current_gripper_position = 0
    if current_gripper_position > 255: current_gripper_position = 255

    # publish gripper position
    gripper_position_publisher.publish(current_gripper_position)

    rate.sleep()
