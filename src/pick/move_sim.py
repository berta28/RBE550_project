#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
import numpy as np
import copy
import json

class robot_simulation():
    """
    Class representing a robot simulation.

    Attributes:
        robot (moveit_commander.RobotCommander): Object to interface with the robot.
        scene (moveit_commander.PlanningSceneInterface): Object to interact with the world.
        arm_group (moveit_commander.MoveGroupCommander): Object representing the manipulator group.
        home_joint_state (list): List of joint values for the home position.
        target_object (dict): Dictionary representing the target object to be picked and placed.
        placed_objects (list): List of names of objects that have been successfully placed.
        planner (str): The ID of the planner to be used for motion planning.

    Methods:
        set_planner(planner): Sets the planner ID for motion planning.
        create_pick_object(object_name): Creates a box in the planning scene at the specified pose.
        move_to_home_position(): Moves the robot to the home position.
        get_all_scene_objects(): Returns the pose of all objects in the planning scene.
        get_scene_object(object_name): Returns the pose of a specific object in the planning scene.
        get_object_pose(object_name): Returns the pose of an object in the planning scene.
        get_grasp_pose(): Returns the target pose for grasping an object.
        get_place_pose(): Returns the target pose for placing an object.
        grasp_object(): Plans and executes the pick motion.
        place_object(): Plans and executes the place motion.
        remove_placed_objects(): Removes all placed objects from the planning scene.
        save_data(success, solving_time, motion_time): Saves data related to the pick operation.
        simulation(pick_name): Performs the simulation of picking and placing an object.
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('moveit_pick_and_place', anonymous=True)
        # Initialize MoveIt! commander
        moveit_commander.roscpp_initialize(sys.argv)
        # Create a RobotCommander object to interface with the robot
        self.robot = moveit_commander.RobotCommander()
        # Create a PlanningSceneInterface object to interact with the world
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self.home_joint_state= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_object = {"name": "table", "size": 0.1}
        self.placed_objects = []

    def set_planner(self, planner):
        """
        Sets the planner ID for motion planning.

        Args:
            planner (str): The ID of the planner.
        """
        self.planner = planner
        self.arm_group.set_planner_id(self.planner)

    def create_pick_object(self, object_name):
        """
        Creates a box in the planning scene at the specified pose.

        Args:
            object_name (str): The name of the object.
        """
        # Create a box in the planning scene at the specified pose
        object_name = object_name  # Replace with the name of your object in MoveIt!
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = 0.4  # Adjust the position as needed
        object_pose.pose.position.y = 0.3
        object_pose.pose.position.z = 0.2  # Height above the table
        object_pose.pose.orientation.x = 0.0  # No rotation
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.0
        object_pose.pose.orientation.w = 1.0
        self.pick_object = {"name": object_name, "size": 0.1}
        self.scene.add_box(object_name, object_pose, size=(0.1, 0.1, 0.1))

    def move_to_home_position(self):
        """
        Moves the robot to the home position.
        """
        # Get the current state of the robot and go to starting position
        self.arm_group.set_joint_value_target(self.home_joint_state)
        # Set the arm and gripper to their respective all_zeros positions
        success, trajectory, solving_time, error_code = self.arm_group.plan()
        self.arm_group.go()
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
    
    def get_all_scene_objects(self):
        """
        Returns the pose of all objects in the planning scene.

        Returns:
            dict: Dictionary containing the pose of each object.
        """
        # Get the pose of the objects in the planning scene (e.g., "box1")
        return self.scene.get_objects()

    def get_scene_object(self, object_name):
        """
        Returns the pose of a specific object in the planning scene.

        Args:
            object_name (str): The name of the object.

        Returns:
            geometry_msgs.msg.Pose: The pose of the object.
        """
        # Get the pose of the objects in the planning scene (e.g., "box1")
        scene_object = self.get_all_scene_objects()
        scene_object = self.get_all_scene_objects()[object_name]
        if scene_object is None:
            rospy.logerr(f"Object '{scene_object}' not found in the planning scene.")
        return scene_object
    
    def get_object_pose(self, object_name):
        """
        Returns the pose of an object in the planning scene.

        Args:
            object_name (str): The name of the object.

        Returns:
            geometry_msgs.msg.Pose: The pose of the object.
        """
        # Get the pose of an object in the planning scene (e.g., "box1")
        return self.get_scene_object(object_name).pose
    

    def get_grasp_pose(self):
        """
        Returns the target pose for grasping an object.

        Returns:
            geometry_msgs.msg.PoseStamped: The target pose for grasping.
        """
        # Define the target pose based on the object's pose
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose = self.get_object_pose(self.pick_object["name"])
        # .1 is the thickness of the box and pose is always at the center of the object
        object_pose_matrix =np.eye(4)
        object_pose_matrix[:3, :3] = R.from_quat([object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w]).as_matrix()
        object_pose_matrix[:3, 3] = [object_pose.position.x, object_pose.position.y, object_pose.position.z]
        # create transform so that target pose can always be at the z-axis of the object and at its surface
        transform_matrix_to_surface = np.eye(4)
        
        transform_matrix_to_surface[2, 3] = self.pick_object["size"]/2 + 0.001
        # robot tip will alway be in -z direction of object
        transform_matrix_rotation = np.eye(4)
        transform_matrix_rotation[:3, :3] = R.from_quat([1,0,0,0]).as_matrix()

        transformed_orientation = object_pose_matrix @ transform_matrix_to_surface @ transform_matrix_rotation

        quaternion = R.from_matrix(transformed_orientation[:3,:3]).as_quat()

        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        target_pose.pose.position.x = transformed_orientation[0, 3]
        target_pose.pose.position.y = transformed_orientation[1, 3]
        target_pose.pose.position.z = transformed_orientation[2, 3]


        return target_pose

    def get_place_pose(self):
        """
        Returns the target pose for placing an object.

        Returns:
            geometry_msgs.msg.PoseStamped: The target pose for placing.
        """
        # Define the target pose based on the object's pose
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose = self.get_object_pose(self.target_object["name"])
        # .1 is the thickness of the box and pose is always at the center of the object
        object_pose_matrix =np.eye(4)
        object_pose_matrix[:3, :3] = R.from_quat([object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w]).as_matrix()
        object_pose_matrix[:3, 3] = [object_pose.position.x, object_pose.position.y, object_pose.position.z]
        # create transform so that target pose can always be at the z-axis of the object and at its surface
        transform_matrix_to_surface = np.eye(4)
        
        transform_matrix_to_surface[2, 3] = self.target_object["size"]/2 + self.pick_object["size"] + 0.001
        # robot tip will alway be in -z direction of object
        transform_matrix_rotation = np.eye(4)
        transform_matrix_rotation[:3, :3] = R.from_quat([1,0,0,0]).as_matrix()

        transformed_orientation = object_pose_matrix @ transform_matrix_to_surface @ transform_matrix_rotation

        quaternion = R.from_matrix(transformed_orientation[:3,:3]).as_quat()

        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        target_pose.pose.position.x = transformed_orientation[0, 3]
        target_pose.pose.position.y = transformed_orientation[1, 3]
        target_pose.pose.position.z = transformed_orientation[2, 3]
        return target_pose
    
    def grasp_object(self):
        """
        Plans and executes the pick motion.
        """
        # Plan and execute the pick motion
        # Define the target pose based on the object's pose
        target_pose = self.get_grasp_pose()
        self.arm_group.set_pose_target(target_pose)
        success, trajectory, solving_time, error_code = self.arm_group.plan()
        if success:
            motion_time = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
            self.arm_group.go()
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            self.scene.attach_box("flange", self.pick_object["name"], touch_links=self.robot.get_link_names(group="manipulator"))
            self.save_data(success, solving_time, motion_time)
        else:
            self.save_data(success, solving_time, 0)
            rospy.logerr("Unable to grab object")
    
    def place_object(self):
        """
        Plans and executes the place motion.
        """
        # Plan and execute the place motion
        # Define the target pose based on the object's pose
        target_pose = self.get_place_pose()
        self.arm_group.set_pose_target(target_pose)
        success, trajectory, solving_time, error_code = self.arm_group.plan()
        if success:
            motion_time = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
            self.arm_group.go()
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            self.arm_group.detach_object(self.pick_object["name"])
            # self.scene.remove_attached_object("flange", self.pick_object["name"])
            self.target_object = copy.deepcopy(self.pick_object)
            self.placed_objects.append(self.pick_object["name"])
            self.save_data(success, solving_time, motion_time)
        else:
            self.save_data(success, solving_time, 0)
            rospy.logerr("Unable to place object")

    def remove_placed_objects(self):
        """
        Removes all placed objects from the planning scene.
        """
        for object_name in self.placed_objects:
            self.scene.remove_world_object(object_name)
        #check if any attached objects
        attached = self.scene.get_attached_objects()
        if attached:
            for object_name in attached:
                self.scene.remove_attached_object("flange", object_name)
                self.scene.remove_world_object(object_name)
        
        self.target_object = {"name": "table", "size": 0.1}
        self.placed_objects = []


    def save_data(self, success, solving_time, motion_time):
        """
        Save the data related to the pick operation.

        Args:
            success (bool): Indicates whether the pick operation was successful.
            solving_time (float): The time taken for solving the pick operation.
            motion_time (float): The time taken for the motion during the pick operation.
        """
        # Create a dictionary to store the data
        data = {
            "object_name": self.pick_object["name"],
            "success": success,
            "solving_time": solving_time,
            "motion_time": motion_time
        }

        try:
            with open("data.json", "r") as file:
                current_data = json.load(file)
        except FileNotFoundError:
            current_data = {}

        if not current_data.get(self.planner):
            current_data[self.planner] = []

        current_data[self.planner].append(data)

        with open("data.json", "w") as file:
            json.dump(current_data, file, indent=4)


    def simulation(self, pick_name):
        """
        Performs the simulation of picking and placing an object.

        Args:
            pick_name (str): The name of the object to be picked.
        """
        # Create the object to pick
        self.create_pick_object(pick_name)
        # Get the object's pose
        # Pick the object
        self.grasp_object()
        # Place the object
        self.place_object()
        # Move to the home position
        self.move_to_home_position()
        # Exit MoveIt
        

if __name__ == '__main__':
    try:
        sim = robot_simulation()
        sim.remove_placed_objects()
        for planner in ["RRT","BITRRT", "TRRT"]:
            sim.set_planner(planner)
            for _ in range(100):
                for i in range(1,4):
                    sim.simulation(f"box_{i}")
                sim.remove_placed_objects()
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("Done")
        
    except rospy.ROSInterruptException:
        sim.remove_placed_objects()
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("Done")