import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Int16

class ExecuteXyzWithGripper():
    def __init__(self) -> None:
        # space mouse data
        self.current_value = (0,0,0,0,0,0)
        self.buttons = (0,0)
        # current gripper target position
        self.current_gripper_position = 0
        # variables for speed adjustment
        self.speed_linear = 0.05
        self.speed_angular = -0.2
        self.gripper_speed = 10

        # initialize caommander to control robot
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()    
        self.group = moveit_commander.MoveGroupCommander("ur3_arm") # real: ur3_arm, gazebo: gripper
        
        # publisher fpr the planned path
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
        
        # publisher for the target gripper position
        self.gripper_position_publisher = rospy.Publisher('/robotiq_gripper/position', Int16, queue_size=10)
        self.spacenav = rospy.Subscriber('/spacenav/joy', Joy, self.spacenav_callback)

        # planning parameters
        self.group.allow_replanning(True)
        self.group.set_pose_reference_frame("world")
        self.group.set_planning_time(1)

        print(self.group.get_current_pose())
        print(self.group.get_current_rpy())

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # create new way point
            waypoints = []
            new_pose = self.calculate_new_pose()
            waypoints.append(copy.deepcopy(new_pose.pose))
            # plan path to new waypoints pose
            plan = self.group.compute_cartesian_path(waypoints, 0.05, 0.0)[0]
            # execute the plan
            self.group.execute(plan, wait=False)
            waypoints.clear()

            # control the gripper
            self.publish_gripper_position()

            self.rate.sleep() 

    def spacenav_callback(self, data):
        # callback function for sapce mouse date
        self.current_value = data.axes  
        self.buttons = data.buttons

    def publish_gripper_position(self):
        # calculate new gripper position
        self.current_gripper_position = self.current_gripper_position + (self.buttons[0] - self.buttons[1])*self.gripper_speed
        # set the value between 0 and 255
        if self.current_gripper_position < 0: self.current_gripper_position = 0
        if self.current_gripper_position > 255: self.current_gripper_position = 255

        # publish gripper position
        self.gripper_position_publisher.publish(self.current_gripper_position)

    def calculate_new_pose(self):
        # get current pose
        new_pose = self.group.get_current_pose()

        # transform new pose linear
        new_pose.pose.position.x = new_pose.pose.position.x + self.current_value[0]*self.speed_linear
        new_pose.pose.position.y = new_pose.pose.position.y + self.current_value[1]*self.speed_linear
        new_pose.pose.position.z = new_pose.pose.position.z + self.current_value[2]*self.speed_linear

        # convert angular values from space mouse into quaternion
        rot_qua = quaternion_from_euler(self.current_value[5]*self.speed_angular,self.current_value[4]*self.speed_angular,self.current_value[3]*self.speed_angular)

        # get angular information from robot
        pose_qua =  [new_pose.pose.orientation.x, new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w]
        
        # transform and build new angular pose 
        new_qua = quaternion_multiply(pose_qua, rot_qua)
        new_pose.pose.orientation.x = new_qua[0]
        new_pose.pose.orientation.y = new_qua[1]
        new_pose.pose.orientation.z = new_qua[2]
        new_pose.pose.orientation.w = new_qua[3]
        return new_pose
    
if __name__ == '__main__':
    execute_xyz = ExecuteXyzWithGripper()