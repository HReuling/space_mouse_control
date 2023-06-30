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
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Int16

current_value = (0,0,0,0,0,0)
buttons = (0,0)
current_gripper_position = 0

def spacenav_callback(data):
    global current_value
    current_value = data.axes  # Beispielhaft wird der Wert der ersten Achse abgerufen
    global buttons 
    buttons = data.buttons
    # print(data.axes[0])

# list_controllers = ControllerLister('/controller_manager') 

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("ur3_arm") # real: ur3_arm, gazebo: gripper
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
gripper_position_publisher = rospy.Publisher('/robotiq_gripper/position', Int16, queue_size=10)

group.allow_replanning(True)
group.set_pose_reference_frame("world")
group.set_planning_time(1)

print(group.get_current_pose())
print(group.get_current_rpy())

speed_linear = 0.05
speed_angular = -0.2
gripper_speed = 10

rate = rospy.Rate(10)  # Veröffentlichungsrate
while not rospy.is_shutdown():
    spacenav = rospy.Subscriber('/spacenav/joy', Joy, spacenav_callback)
    waypoints = []
    new_pose = group.get_current_pose()
    # new_rpy = group.get_current_rpy()
    new_rpy = group.get_current_rpy()
    # print(new_pose)
    new_pose.pose.position.x = new_pose.pose.position.x + current_value[0]*speed_linear
    new_pose.pose.position.y = new_pose.pose.position.y + current_value[1]*speed_linear
    new_pose.pose.position.z = new_pose.pose.position.z + current_value[2]*speed_linear

    #(new_rpy[0],new_rpy[1],new_rpy[2]) = euler_from_quaternion([new_pose.pose.orientation.x, new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w])

    # uebersprung bei 2 pi einbeziehen
    # new_rpy[0] = new_rpy[0] + current_value[5]*speed_angular
    # new_rpy[1] = new_rpy[1] + current_value[3]*-speed_angular
    # new_rpy[2] = new_rpy[2] + current_value[4]*speed_angular

    rot_qua = quaternion_from_euler(current_value[5]*speed_angular,current_value[4]*speed_angular,current_value[3]*speed_angular)
    #print (new_qua)
    pose_qua =  [new_pose.pose.orientation.x, new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w]

    new_qua = quaternion_multiply(pose_qua, rot_qua)

    # (new_pose.pose.orientation.x,new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w) = quaternion_from_euler(new_rpy[0],new_rpy[1],new_rpy[2])
    new_pose.pose.orientation.x = new_qua[0]
    new_pose.pose.orientation.y = new_qua[1]
    new_pose.pose.orientation.z = new_qua[2]
    new_pose.pose.orientation.w = new_qua[3]
    #(new_pose.pose.orientation.x,new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w) = quaternion_from_euler(current_value[0],current_value[1],current_value[2])
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