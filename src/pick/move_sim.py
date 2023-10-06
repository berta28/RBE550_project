#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

def main():
    # Initialize ROS node
    rospy.init_node('moveit_pick_and_place', anonymous=True)
    
    # Initialize MoveIt! commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Create a RobotCommander object to interface with the robot
    robot = moveit_commander.RobotCommander()
    
    # Create a PlanningSceneInterface object to interact with the world
    scene = moveit_commander.PlanningSceneInterface()

    # Define the object to pick and its pose
    object_name = "box_1"  # Replace with the name of your object in MoveIt!
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = robot.get_planning_frame()
    object_pose.pose.position.x = 0.4  # Adjust the position as needed
    object_pose.pose.position.y = 0.3
    object_pose.pose.position.z = 0.2  # Height above the table
    object_pose.pose.orientation.w = 1.0  # No rotation
    scene.add_box(object_name, object_pose, size=(0.1, 0.1, 0.1))
    # Create a MoveGroupCommander for the arm and gripper
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    arm_group.stop()
    arm_group.clear_pose_targets()
    # Get the current state of the robot and go to starting position

    home_joint_state= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    arm_group.set_joint_value_target(home_joint_state)
    # Set the arm and gripper to their respective all_zeros positions
    plan = arm_group.plan()
    arm_group.go()
  
    # Get the pose of the object in the planning scene (e.g., "box1")
    all_objects = scene.get_objects()
    # object_poses = scene.get_object_poses(all_objects)

    if object_pose is None:
        rospy.logerr(f"Object '{object_name}' not found in the planning scene.")
        exit()

    # Define the target pose based on the object's pose
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = robot.get_planning_frame()
    target_pose.pose = all_objects[object_name].pose
    target_pose.pose.position.z = target_pose.pose.position.z + .06
    target_pose.pose.orientation.x = 1
    target_pose.pose.orientation.w = 0 

    # Plan to the target pose
    arm_group.set_pose_target(target_pose)
    # Set the arm and gripper to their respective all_zeros positions

    plan = arm_group.plan()
    arm_group.go()

    scene.attach_box("flange", object_name, touch_links=robot.get_link_names(group="manipulator"))
    
    # Plan and execute the pick motion
    # Define the target pose based on the object's pose
    target_pose.pose = all_objects["table"].pose
    # .06 is just above the collision of the table and .1 is the thikness of the box
    target_pose.pose.position.z = target_pose.pose.position.z + .06 + .1
    target_pose.pose.orientation.x = 1
    target_pose.pose.orientation.w = 0
    arm_group.set_pose_target(target_pose)  # Define your pick pose
    plan = arm_group.plan()
    arm_group.go()
    scene.remove_attached_object("", object_name)
    
    arm_group.set_joint_value_target(home_joint_state)
    # # Set the arm and gripper to their respective all_zeros positions
    plan = arm_group.plan()
    # arm_group.execute(plan)
    arm_group.go()
    # Clean up and exit
    arm_group.stop()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass