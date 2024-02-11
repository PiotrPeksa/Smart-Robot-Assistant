#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from moveit_commander.conversions import pose_to_list
import sys
import actionlib

class RobotArmController:
    def __init__(self):
        rospy.init_node('robot_arm_controller', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.door_status_sub = rospy.Subscriber("/door_status", String, self.door_status_callback)
        self.glass_pose_sub = rospy.Subscriber("/object_pose", PoseStamped, self.glass_pose_callback)

    def move_to_target_pose(self, target_pose):
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()

        if plan:
            self.arm_group.execute(plan)
        else:
            rospy.logerr("Failed to plan a trajectory.")

    def execute_door_open_action(self):
        goal = GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 1.0
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        result = self.gripper_client.get_result()
        if result:
            rospy.loginfo("Door opened.")
        else:
            rospy.logerr("Failed to open the door.")
    
    def execute_glass_pickup_action(self):
        goal = GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 1.0
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        result = self.gripper_client.get_result()
        if result:
            rospy.loginfo("Glass picked up.")
        else:
            rospy.logerr("Failed to pick up the glass.")

    def open_door(self):
        # Implement logic to open the door using the robotic arm and perception data
        door_handle_pose = PoseStamped()
        door_handle_pose.header.frame_id = "map"  # Replace with the appropriate frame
        door_handle_pose.pose.position.x = 1.0  # Adjust as needed
        door_handle_pose.pose.position.y = 0.5  # Adjust as needed
        door_handle_pose.pose.position.z = 0.9  # Adjust as needed
        door_handle_pose.pose.orientation.x = 0.0
        door_handle_pose.pose.orientation.y = 0.0
        door_handle_pose.pose.orientation.z = 0.0
        door_handle_pose.pose.orientation.w = 1.0

        # Plan and execute the motion to reach the door handle


        self.move_to_target_pose(door_handle_pose)


    # Execute the action to open the door (implement your door opening mechanism)
        self.execute_door_open_action() 
    
    def pick_up_glass(self):
            
            # Implement logic to pick up a glass using the robotic arm and perception data
            # similar to the comments in the open_door function
            glass_pose = PoseStamped()
            glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
            # Set the target position and orientation of the glass
            # glass_pose.pose.position.x = 0.8  # Adjust as needed
            glass_pose.pose.position.y = 0.2  # Adjust as needed
            glass_pose.pose.position.z = 0.7  # Adjust as needed
            glass_pose.pose.orientation.x = 0.0
            glass_pose.pose.orientation.y = 0.0
            glass_pose.pose.orientation.z = 0.0
            glass_pose.pose.orientation.w = 1.0
            # Plan and execute the motion to reach the glass
            self.move_to_target_pose(glass_pose)

            # Execute the action to pick up the glass (implement your gripper control)
            # self.execute_glass_pickup_action() #needs to be written
            # This function should contain the control logic for your robot's gripper or end-effector to pick up a glass.
            # Implement logic to pick up a glass using the robotic arm and perception data
            # similar to the comments in the open_door function
            glass_pose = PoseStamped()
            glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
            # Set the target position and orientation of the glass
            # glass_pose.pose.position.x = 0.8  # Adjust as needed
            glass_pose.pose.position.y = 0.2  # Adjust as needed
            glass_pose.pose.position.z = 0.7  # Adjust as needed
            glass_pose.pose.orientation.x = 0.0
            glass_pose.pose.orientation.y = 0.0
            glass_pose.pose.orientation.z = 0.0
            glass_pose.pose.orientation.w = 1.0
            # Plan and execute the motion to reach the glass
            self.move_to_target_pose(glass_pose)

            # Execute the action to pick up the glass (implement your gripper control)
            # self.execute_glass_pickup_action() #needs to be written
            # This function should contain the control logic for your robot's gripper or end-effector to pick up a glass.
            # Implement logic to pick up a glass using the robotic arm and perception data
            # similar to the comments in the open_door function
            glass_pose = PoseStamped()
            glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
            # Set the target position and orientation of the glass
            # glass_pose.pose.position.x = 0.8  # Adjust as needed
            glass_pose.pose.position.y = 0.2  # Adjust as needed
            glass_pose.pose.position.z = 0.7  # Adjust as needed
            glass_pose.pose.orientation.x = 0.0
            glass_pose.pose.orientation.y = 0.0
            glass_pose.pose.orientation.z = 0.0
            glass_pose.pose.orientation.w = 1.0
            # Plan and execute the motion to reach the glass
            self.move_to_target_pose(glass_pose)

            # Execute the action to pick up the glass (implement your gripper control)
            # self.execute_glass_pickup_action() #needs to be written
            # This function should contain the control logic for your robot's gripper or end-effector to pick up a glass.
            # Implement logic to pick up a glass using the robotic arm and perception data
            # similar to the comments in the open_door function
            glass_pose = PoseStamped()
            glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
            # Set the target position and orientation of the glass
            # glass_pose.pose.position.x = 0.8  # Adjust as needed
            glass_pose.pose.position.y = 0.2  # Adjust as needed
            glass_pose.pose.position.z = 0.7  # Adjust as needed
            glass_pose.pose.orientation.x = 0.0
            glass_pose.pose.orientation.y = 0.0
            glass_pose.pose.orientation.z = 0.0
            glass_pose.pose.orientation.w = 1.0
            # Plan and execute the motion to reach the glass
            self.move_to_target_pose(glass_pose)

            # Execute the action to pick up the glass (implement your gripper control)
            # self.execute_glass_pickup_action() #needs to be written
            # This function should contain the control logic for your robot's gripper or end-effector to pick up a glass.
            # Implement logic to pick up a glass using the robotic arm and perception data
            # similar to the comments in the open_door function
            glass_pose = PoseStamped()
            glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
            # Set the target position and orientation of the glass
            # glass_pose.pose.position.x = 0.8  # Adjust as needed
            glass_pose.pose.position.y = 0.2  # Adjust as needed
            glass_pose.pose.position.z = 0.7  # Adjust as needed
            glass_pose.pose.orientation.x = 0.0
            glass_pose.pose.orientation.y = 0.0
            glass_pose.pose.orientation.z = 0.0
            glass_pose.pose.orientation.w = 1.0
            # Plan and execute the motion to reach the glass
            self.move_to_target_pose(glass_pose)

            # Execute the action to pick up the glass (implement your gripper control)
            # self.execute_glass_pickup_action() #needs to be written
            # This function should contain the control logic for your robot's gripper or end-effector to pick up a glass.
            # Implement logic to pick up a glass using the robotic arm and perception data
            # similar to the comments in the open_door function
            glass_pose = PoseStamped()
            glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
            # Set the target position and orientation of the glass
            # glass_pose.pose.position.x = 0.8  # Adjust as needed
            glass_pose.pose.position.y = 0.2  # Adjust as needed
            glass_pose.pose.position.z = 0.7  # Adjust as needed
            glass_pose.pose.orientation.x = 0.0
            glass_pose.pose.orientation.y = 0.0
            glass_pose.pose.orientation.z = 0.0
            glass_pose.pose.orientation.w = 1.0
            # Plan and execute the motion to reach the glass
            self.move_to_target_pose(glass_pose)

            # Execute the action to pick up the glass (implement your gripper control)
            # self.execute_glass_pickup_action() #needs to be written
            # This function should contain the control logic for your robot's gripper or end-effector to pick up a glass.

    def move_to_target_pose(self, target_pose):
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()

        if plan:
            self.arm_group.execute(plan)
        else:
            rospy.logerr("Failed to plan a trajectory.")

    def open_door(self):

        door_handle_pose = PoseStamped()
        door_handle_pose.header.frame_id = "map"  # Replace with the appropriate frame
        door_handle_pose.pose.position.x = 1.0  # Adjust as needed
        door_handle_pose.pose.position.y = 0.5  # Adjust as needed
        door_handle_pose.pose.position.z = 0.9  # Adjust as needed
        door_handle_pose.pose.orientation.x = 0.0
        door_handle_pose.pose.orientation.y = 0.0
        door_handle_pose.pose.orientation.z = 0.0
        door_handle_pose.pose.orientation.w = 1.0

        # Plan and execute the motion to reach the door handle
        self.move_to_target_pose(door_handle_pose)

    def pick_up_glass(self):
        # Implement logic to pick up a glass using the robotic arm and perception data
        # similar to the comments in the open_door function
        glass_pose = PoseStamped()
        glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
        # Set the target position and orientation of the glass
        # glass_pose.pose.position.x = 0.8  # Adjust as needed
        glass_pose.pose.position.y = 0.2  # Adjust as needed
        glass_pose.pose.position.z = 0.7  # Adjust as needed
        glass_pose.pose.orientation.x = 0.0
        glass_pose.pose.orientation.y = 0.0
        glass_pose.pose.orientation.z = 0.0
        glass_pose.pose.orientation.w = 1.0
        # Plan and execute the motion to reach the glass
        self.move_to_target_pose(glass_pose)

        # Execute the action to pick up the glass (implement your gripper control)
        # self.execute_glass_pickup_action() #needs to be written
        # This function should contain the control logic for your robot's gripper or end-effector to pick up a glass.
        glass_pose.pose.orientation.y = 0.0
        glass_pose.pose.orientation.z = 0.0
        glass_pose.pose.orientation.w = 1.0

    # Plan and execute the motion to reach the glass
    def move_to_target_pose(self, target_pose):
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()

        if plan:
            self.arm_group.execute(plan)
        else:
            rospy.logerr("Failed to plan a trajectory.")

    def open_door(self):
        door_handle_pose = PoseStamped()
        door_handle_pose.header.frame_id = "map"  # Replace with the appropriate frame
        door_handle_pose.pose.position.x = 1.0  # Adjust as needed
        door_handle_pose.pose.position.y = 0.5  # Adjust as needed
        door_handle_pose.pose.position.z = 0.9  # Adjust as needed
        door_handle_pose.pose.orientation.x = 0.0
        door_handle_pose.pose.orientation.y = 0.0
        door_handle_pose.pose.orientation.z = 0.0
        door_handle_pose.pose.orientation.w = 1.0

        # Plan and execute the motion to reach the door handle
        self.move_to_target_pose(door_handle_pose)

        # Execute the action to open the door (implement your door opening mechanism)
        self.execute_door_open_action()  # needs to be written

    def pick_up_glass(self):
        glass_pose = PoseStamped()
        glass_pose.header.frame_id = "map"  # Replace with the appropriate frame
        glass_pose.pose.position.x = 0.8  # Adjust as needed
        glass_pose.pose.position.y = 0.2  # Adjust as needed
        glass_pose.pose.position.z = 0.7  # Adjust as needed
        glass_pose.pose.orientation.x = 0.0
        glass_pose.pose.orientation.y = 0.0
        glass_pose.pose.orientation.z = 0.0
        glass_pose.pose.orientation.w = 1.0

        # Plan and execute the motion to reach the glass
        self.move_to_target_pose(glass_pose)

        # Execute the action to pick up the glass (implement your gripper control)
        self.execute_glass_pickup_action()  # needs to be written

    def door_status_callback(self, msg):
        if msg.data == "open":
            self.open_door()

    def glass_pose_callback(self, msg):
        if msg.pose.position.x < 1.0:  # Replace with an appropriate threshold
            self.pick_up_glass()

if __name__ == '__main__':
    try:
        controller = RobotArmController()
        target_pose_door = PoseStamped()
        target_pose_door.header.frame_id = "map"  # we gotta with the appropriate frame
        # Set the target pose for the door location
        

        target_pose_glass = PoseStamped()
        target_pose_glass.header.frame_id = "map"  # we gotta with the appropriate frame
        # Set the target pose for the glass location
        

        controller.move_to_target_pose(target_pose_door)
        controller.move_to_target_pose(target_pose_glass)

    except rospy.ROSInterruptException:
        pass
