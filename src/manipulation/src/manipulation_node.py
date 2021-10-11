#!/usr/bin/env python

# standard ROS imports
import sys
import time
import math
import rospy
import copy
from std_msgs.msg import ColorRGBA, Float32, String
import tf2_ros
import tf
import tf_conversions
import numpy as np
import os
from subprocess import Popen
# move_arm imports
import geometry_msgs.msg 
import moveit_msgs.msg 
import control_msgs.msg  
import trajectory_msgs.msg 
# move_head imports
from control_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib # also used for move_base to pose
# local modules for Fetch primitives
from move_arm.fetch_move_arm import *
from move_head.fetch_move_head import *
from barista_planning_scene import * 
from sound.fetch_sound import * 
# vision imports
from ar_track_alvar_msgs.msg import AlvarMarkers


############################################################################################################
def main():
    rospy.init_node('manipulation')

    # Run required nodes as subprocesses
    Popen("roslaunch darknet_ros darknet_ros.launch", shell=True)
    rospy.sleep(1)
    Popen("roslaunch ar_track_alvar fetch_indiv.launch", shell=True)
    rospy.sleep(1)
    Popen("rosrun perception bb_vision", shell=True)
    rospy.sleep(1)

    input("\nEnsure object can be seen. Press enter to continue.")
    # Create class and finde scene objects
    robot = RobotManipulation()
    (machine_location, bottleloc) = robot.plan.marker_locations()
    global cup_location # required for various functions
    cup_location = robot.plan.find_cup_location()

    # Kill all vision nodes, ensuring enough processing power for movements
    os.system("rosnode kill /darknet_ros")
    os.system("rosnode kill /ar_track_alvar")
    os.system("rosnode kill /vision")
    rospy.sleep(1)

    # Arm tuck if required
    #robot.arm.tuck_arm(True)

    #########-------------------  Manipulations  -------------------#########
    # 1. Pick up cup
    eef_pos = robot.pick_cup()

    # 2. Move cup to machine
    cup_in_machine_pos = robot.move_to_machine(eef_pos)

    # 3. Push button
    coffee_size = False # false = small, true=large
    robot.push_button(machine_location, coffee_size)

    # 4. Remove cup from machine
    milk_requirement = True # specify if milk is required
    cup_on_table_pos = robot.remove_cup_from_machine(cup_in_machine_pos, milk_requirement, coffee_size)

    # 5. Place cup in bracket
    robot.ready_for_transport(cup_on_table_pos)


################------------- Robot manipulation Class -------------################
class RobotManipulation(object):
    def __init__(self):
        self.arm = FetchArm()
        self.plan = RobotPlanningScene()
        self.sound = FetchSound()

        # Commander planning config
        self.arm.move_commander.set_planner_id("RRTConnectkConfigDefault")
        #self.arm.move_commander.set_planner_id("TRRTkConfigDefault")
        self.arm.move_commander.set_planning_time(15)
        self.arm.move_commander.allow_replanning(True)
        self.arm.move_commander.set_num_planning_attempts(5)

    ################------------- Manipulation functions -------------################
    def pick_cup(self):
        """ Function to pick up a cup. """
        self.sound.talk("Picking up cup.")

        # Pre grasp approach
        eef_pos=[cup_location[0]- APPROACH_DISTANCE*3, cup_location[1],cup_location[2]]
        cup_pre_grasp_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(cup_pre_grasp_pose)
        # Make sure gripper is open
        self.arm.gripper.open_gripper()
        self.plan.remove_cup_object()
        rospy.sleep(0.5)

        # Approach
        self.arm.init_upright_constraint(10,"x")
        self.plan.remove_cup_object()
        eef_pos[0] = cup_location[0]
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(0.5)
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)

        # Lift up
        eef_pos[2]+=0.05
        self.arm.cartesian_path(eef_pos[2],"z")
        rospy.sleep(0.5)
        self.plan.attach_cup()

        # Move back
        eef_pos[0]-=APPROACH_DISTANCE*3
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(0.5)

        return eef_pos
        
    def move_to_machine(self, eef_pos):
        """ Function to move cup to machine. """
        self.arm.init_upright_constraint(10,"x")

        eef_pos = machine_location[:]

        # Move accross to machine
        self.arm.cartesian_path(eef_pos[1],'y')
        rospy.sleep(0.5)

        # move forward
        self.plan.remove_machine_object()
        eef_pos[0] -= CUP_TO_MACHINE_OFFSET
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(0.5)

        # place cup
        self.arm.move_torso_relative(-0.02)   # Adjust z
        cup_in_machine = eef_pos[:] # copy values
        self.arm.gripper.open_gripper()
        self.plan.deposit_cup()
        rospy.sleep(0.5)

        # move gripper back
        eef_pos[0]-= APPROACH_DISTANCE*3
        self.arm.cartesian_path(eef_pos[0],"x")   
        rospy.sleep(0.5)

        # Add machine back
        self.plan.add_machine_object()
        self.plan.planning_scene.addCylinder('cup_in_machine',0.13,0.05,cup_in_machine[0],cup_in_machine[1],cup_in_machine[2])

        self.arm.move_commander.clear_path_constraints()
        return cup_in_machine

    def open_hatch(self):
        """ Function to open hatch. NOT USED OR TESTED ON ROBOT"""
        # Tilt gripper and move to hatch
        grasp_angle = [np.deg2rad(90),0,0]
        eef_pos[2]+=0.16
        tilt_grip=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(tilt_grip)   
        rospy.sleep(1)

        #forward
        eef_pos[0]+=0.05
        #eef_pos[2]+=0.03
        hatch_pose=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(hatch_pose)   
        rospy.sleep(1)

        #self.arm.gripper.close_gripper(0.05)
        grasp_angle = [np.deg2rad(90),np.deg2rad(-10),0]
        hatch_pose=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(hatch_pose) 

    def push_button(self, marker_loc, coffee_size):
        """ Function to push button. """

        # Pod instructions
        self.sound.talk("Please place a pod in the hatch.")
        self.sound.talk("I will wait 5 seconds")
        rospy.sleep(5)

        self.sound.talk("Pushing button now")
        eef_pos = [0]*3

        # Close gripper
        self.arm.gripper.close_gripper(0.027)

        # Pre-button location
        if coffee_size: #large
            eef_pos[1] = marker_loc[1] - 0.15 # right button
            grasp_angle=[np.deg2rad(-45),0,0]
        else: #small
            eef_pos[1] = marker_loc[1] + 0.15 #left button
            grasp_angle = [np.deg2rad(45),0,0]

        eef_pos[0] = marker_loc[0]
        eef_pos[2] = marker_loc[2] + 0.3
        pre_pre_button_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_pre_button_pose)
        self.plan.remove_machine_button_objects()
        rospy.sleep(0.5)

        # Button location
        eef_pos[0] = marker_loc[0] + 0.01
        if coffee_size: #large
            eef_pos[1] = marker_loc[1] - 0.12
        else: #small
            eef_pos[1] = marker_loc[1] + 0.11
        eef_pos[2] = marker_loc[2] + 0.225
        pre_button_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_button_pose)
        rospy.sleep(1)
        
        # Push button
        self.arm.gripper.close_gripper(0.085)
        rospy.sleep(0.5)
        self.arm.gripper.close_gripper(0.027)

        # Retreat
        eef_pos[0]-=APPROACH_DISTANCE*3
        self.arm.cartesian_path(eef_pos[0],"x")
        self.plan.add_machine_button_objects()

    def remove_cup_from_machine(self,cup_in_machine_pos, milk_requirement, coffee_size):
        """ Removes coffee, moves to milk function if required, and places cup back on table """

        eef_pos = cup_in_machine_pos[:] # copy values

        # Open gripper
        self.arm.gripper.open_gripper()

        # Pre-approach
        eef_pos[0]-=APPROACH_DISTANCE
        pre_cup_locatione = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]+MACHINE_TO_MARKER_Z_OFFSET), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_cup_locatione)

        # Sleep to wait for coffee dispensing
        if coffee_size: #large
            rospy.sleep(35)
        else: #small
            rospy.sleep(10)
            
        # Approach
        self.arm.init_upright_constraint(10,"x")
        eef_pos[0]+=APPROACH_DISTANCE
        self.plan.remove_machine_object()
        self.arm.cartesian_path(eef_pos[0],"x")     
        rospy.sleep(0.5)

        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.attach_cup()

        # move gripper back
        eef_pos[0]-=APPROACH_DISTANCE*3
        self.arm.cartesian_path(eef_pos[0],"x") 

        # Add machine back
        self.plan.add_machine_object()
        rospy.sleep(0.5)

        # Add milk if required
        if milk_requirement:
            self.get_milk()

        # Move back to original position
        self.arm.init_upright_constraint(11,"x")

        eef_pos = cup_location[:]
        back_on_table = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]+0.02), 
                euler_to_quat(grasp_angle) )
        self.arm.solve_path_plan(back_on_table)

        # Place back on table
        self.plan.deposit_cup()
        rospy.sleep(0.5)
        self.arm.move_torso_relative(-0.04) 

        cup_on_table_pos = eef_pos[:]
        rospy.sleep(0.5)

        #Open gripper
        self.arm.gripper.open_gripper()

        # Move back
        self.arm.init_upright_constraint(10,'x')
        eef_pos[0]-=APPROACH_DISTANCE*2
        self.arm.cartesian_path(eef_pos[0],"x")
        self.plan.add_cup_object()

        self.sound.talk("Please screw the lid on the cup...")
        rospy.sleep(10)

        return cup_on_table_pos

    def get_milk(self):
        """ Moves cup to get milk if required """
        self.arm.move_commander.clear_path_constraints()
        self.arm.init_upright_constraint(11,'x')

        eef_pos = milk_location[:]

        # Pre-approach milk
        eef_pos[0]-=2*APPROACH_DISTANCE
        eef_pos[2]+=0.15
        milk_grasp_angle = [0,np.deg2rad(10),0]
        milk_pre_approach= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(milk_grasp_angle)) 
        self.arm.solve_path_plan(milk_pre_approach)
        rospy.sleep(0.5)

        # Approach milk
        self.plan.remove_milk_object()

        eef_pos[0] += APPROACH_DISTANCE*1.3
        self.arm.cartesian_path(eef_pos[0],'x')
        rospy.sleep(5)

        # Retreat milk
        eef_pos[0] -= APPROACH_DISTANCE*1.5
        self.arm.cartesian_path(eef_pos[0],'x')
        self.plan.add_milk_object()

    def ready_for_transport(self,cup_on_table_pos):
        """ Function to place cup on base for transport """
        self.arm.move_commander.clear_path_constraints()
        self.arm.move_commander.clear_pose_targets()
        self.sound.talk("Placing cup in base holder now.")
        top_grasp_angle = [0,np.deg2rad(90),0]
        self.arm.gripper.open_gripper()

        # Pre-grasp approach
        eef_pos = cup_on_table_pos[:]
        eef_pos[0]+=0.015
        eef_pos[2]=0.96 #NB: DEPENDANT ON CUP
        eef_pos[2]+=2*APPROACH_DISTANCE
        transport_grasp = Pose( Point(eef_pos[0], 
            eef_pos[1], 
            eef_pos[2]), 
            euler_to_quat(top_grasp_angle)) 
        self.arm.move_gripper_to_pose(transport_grasp)
        self.plan.planning_scene.removeCollisionObject("cup")
        rospy.sleep(0.5)
        
        self.sound.talk("Moving cup to base holder")

        # Grasp
        self.arm.move_torso_relative(-APPROACH_DISTANCE*1.8)
        rospy.sleep(0.5)
        # Close Gripper
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Retreat
        self.arm.move_torso_relative(APPROACH_DISTANCE)
        # Attach cup
        self.plan.attach_cup_top()

        # Init orientation constraint
        self.arm.init_upright_constraint(10,"z")
        self.sound.talk("Please wait while I plan my movemnts........ I don't want to spill the coffee!")
        
        # Pre-approach base
        eef_pos = cup_holder[:]
        eef_pos[2] += 4*APPROACH_DISTANCE
        base_pre_approach= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(top_grasp_angle)) 
        self.arm.solve_path_plan(base_pre_approach)
        rospy.sleep(0.5)

        # Remove holder collision object
        self.plan.planning_scene.removeCollisionObject("cup_holder")

        # Place in bracket
        self.arm.move_torso_relative(-APPROACH_DISTANCE*2.3)

        rospy.sleep(0.5)
        self.arm.gripper.open_gripper()
        self.plan.planning_scene.removeAttachedObject("gripped_cup")
        self.plan.planning_scene.removeCollisionObject("gripped_cup")

        # Retreat
        self.arm.move_torso_relative(APPROACH_DISTANCE*2)

        # Add back collision object
        self.plan.planning_scene.addCylinder("cup_holder",0.2,0.05,cup_holder[0],cup_holder[1],cup_holder[2])

        # Tuck arm for transport
        self.arm.move_commander.clear_path_constraints()
        rospy.sleep(1)
        self.arm.tuck_arm(True)
        self.sound.talk("I am ready to deliver the coffeeeeee!")



#####################################################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass







    