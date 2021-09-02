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
from path_planning import * 
# vision imports
from ar_track_alvar_msgs.msg import AlvarMarkers


############################################################################################################
def main():
    rospy.init_node('Manipulation')

    # Create class objects
    robotManipulation = RobotManipulation()
    (machine_loc,bottleloc) = robotManipulation.plan.marker_locations()

    robotManipulation.arm.tuck_arm(True)

    #####--- Manipulations ---#####
    # 1. Pick up cup
    #eef_pos = robotManipulation.pick_cup()

    # 2. Move cup to machine
    #cup_in_machine_pos = robotManipulation.move_to_machine(eef_pos)

    # 3. Push button
    #robotManipulation.push_button(machine_loc)

    # 4. Remove cup from machine
    #robotManipulation.remove_cup_from_machine(cup_in_machine_pos)

    # 5. Place cup in bracket
    #robotManipulation.ready_for_transport()

    #robotManipulation.get_milk()


################------------- Robot manipulation Class -------------################
class RobotManipulation(object):
    def __init__(self):
        self.arm = FetchArm()
        self.plan = RobotPathPlanning()

        # Commander planning config
        self.arm.move_commander.set_planner_id("RRTConnectkConfigDefault")
        #self.arm.move_commander.set_planner_id("TRRTkConfigDefault")
        self.arm.move_commander.set_planning_time(15)
        self.arm.move_commander.allow_replanning(True)
        self.arm.move_commander.set_num_planning_attempts(5)

    ################------------- Manipulation functions -------------################
    def pick_cup(self):
        """ Function to pick up a cup. """
        rospy.loginfo("Pick Cup")

        #rospy.sleep(8)
        #self.head.tilt_eyes({'direction':"down"}) # Look Toward Objects
        #rate = rospy.Rate(5)

        #transCup = self.tfBuffer.lookup_transform('base_link', 'BJ_item', rospy.Time())
        """ try:
            transCup = tfBuffer.lookup_transform('BJ_item', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("")
            rate.sleep() """


        """cup_pos[0] = transCup.transform.translation.x
        cup_pos[1] = transCup.transform.translation.y
        cup_pos[2] = transCup.transform.translation.z"""

        rospy.loginfo("cup 1 (x,y,z) = %s,%s,%s", cup_pos[0], cup_pos[1], cup_pos[2])

        # Pre grasp approach
        eef_pos=[cup_pos[0]- APPROACH_DISTANCE*3, cup_pos[1],cup_pos[2]]
        rospy.loginfo("Pre-grasp")
        cup_pre_grasp_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(cup_pre_grasp_pose)
        #rospy.loginfo(eef_pos[0])
        # Make sure gripper is open
        self.arm.gripper.open_gripper()
        self.plan.planning_scene.removeCollisionObject("cup_1")
        rospy.sleep(0.5)

        # Approach
        rospy.loginfo("Approach")
        eef_pos[0] = cup_pos[0]
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(1)
        
        # Close Gripper
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.attach_cup()

        # Lift up
        #torso = self.arm.getJointPosition("torso_lift_joint")
        #self.arm.move_torso(torso+0.06)
        eef_pos[2]+=0.05
        lift = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(lift)
        rospy.sleep(0.5)

        # Move back
        eef_pos[0]-=APPROACH_DISTANCE*2
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(0.5)

        return eef_pos
        
    def move_to_machine(self, eef_pos):
        """ Function to move cup to machine. """
        rospy.loginfo("Move to machine")
        self.arm.init_upright_constraint(10,"x")

        eef_pos = machine_location[:]

        # Move accross to machine
        self.arm.cartesian_path(eef_pos[1],'y')
        rospy.sleep(0.5)

        # Adjust z
        self.arm.cartesian_path(eef_pos[2]+MACHINE_TO_MARKER_Z_OFFSET,'z')

        # move forward
        self.plan.remove_machine_object()
        eef_pos[0] -= CUP_TO_MACHINE_OFFSET
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(0.5)

        # place cup
        cup_in_machine = eef_pos[:] # copy values
        self.arm.gripper.open_gripper()
        self.plan.deposit_cup()
        rospy.sleep(0.5)

        # move gripper back
        eef_pos[0]-= APPROACH_DISTANCE*3
        retreat_machine=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle))
        self.arm.move_gripper_to_pose(retreat_machine)   
        rospy.sleep(0.5)

        # Add machine back
        self.plan.add_machine_object()
        
        self.arm.move_commander.clear_path_constraints()
        return cup_in_machine

    def open_hatch(self):
        """ Function to open hatch. """
        #eef_pos = self.get_eef_pos()

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

    def push_button(self, marker_loc):
        """ Function to push button. """
        eef_pos = [0]*3

        # Close gripper
        self.arm.gripper.close_gripper(0.027)

        # Pre-button
        eef_pos[0] = marker_loc[0]
        eef_pos[1] = marker_loc[1] - 0.10
        eef_pos[2] = marker_loc[2] + 0.23

        #self.arm.cartesian_path(eef_pos[1]-0.1,"y")

        grasp_angle=[np.deg2rad(-45),0,0]
        pre_button_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_button_pose)
        rospy.sleep(1)
        
        self.arm.gripper.close_gripper(0.085)
        rospy.sleep(0.5)
        self.arm.gripper.close_gripper(0.027)

        # Retreat
        eef_pos[0]-=APPROACH_DISTANCE*3
        self.arm.cartesian_path(eef_pos[0],"x")

    def remove_cup_from_machine(self,cup_in_machine_pos):
        """ Removes coffee, moves to milk function if required, and places cup back on table """
        self.arm.init_upright_constraint(7,"x")
        eef_pos = cup_in_machine_pos[:] # copy values

        # Open gripper
        self.arm.gripper.open_gripper()

        # Pre-approach
        eef_pos[0]-=APPROACH_DISTANCE
        pre_cup_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]+MACHINE_TO_MARKER_Z_OFFSET), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_cup_pose)

        # Approach
        eef_pos[0]+=APPROACH_DISTANCE
        self.plan.planning_scene.removeCollisionObject("Machine")
        self.arm.cartesian_path(eef_pos[0],"x")     
        rospy.sleep(0.5)

        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.attach_cup()

        # move gripper back
        eef_pos[0]-=APPROACH_DISTANCE
        self.arm.cartesian_path(eef_pos[0],"x") 

        # Add machine back
        self.plan.add_machine_object()

        # TODO: Move to milk function if required
        #self.get_milk()

        # TODO: Need to change following movements post getting milk

        # Move accross back to original position
        eef_pos=[cup_pos[0] - APPROACH_DISTANCE, cup_pos[1], cup_pos[2]]
        self.arm.cartesian_path(eef_pos[1],"y")
        rospy.sleep(0.5)

        # Place back on table
        eef_pos[0]+=APPROACH_DISTANCE
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(1)

        # Place cup on table with coffee in it!
        rospy.loginfo("placing on table")
        place_coffee_on_table = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]+0.01), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(place_coffee_on_table)
        self.plan.deposit_cup()
        rospy.sleep(0.5)

        #Open gripper
        self.arm.gripper.open_gripper()

        # Move back
        eef_pos[0]-=APPROACH_DISTANCE
        self.arm.cartesian_path(eef_pos[0],"x")
        self.plan.planning_scene.addCylinder("cup_1", 0.1, 0.05, cup_pos[0], cup_pos[1], cup_pos[2])

    def get_milk(self):
        """ Moves cup to get milk if required """
        self.arm.move_commander.clear_path_constraints()
        self.arm.init_upright_constraint(7,'x')

        eef_pos = milk_location[:]

        # Pre-approach milk
        eef_pos[0]-=2*APPROACH_DISTANCE
        eef_pos[2]+=0.18
        milk_pre_approach= Pose( Point(eef_pos[0]+0.02, 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.solve_path_plan(milk_pre_approach)
        rospy.sleep(0.5)

        # Approach milk
        self.plan.remove_milk_object()

        eef_pos[0] += APPROACH_DISTANCE*1.5
        self.arm.cartesian_path(eef_pos[0],'x')

        # TODO: calculate amount of milk required 
        rospy.sleep(2)

        # Retreat milk
        eef_pos[0] -= APPROACH_DISTANCE*1.5
        self.arm.cartesian_path(eef_pos[0],'x')
        self.plan.add_milk_object()

    def ready_for_transport(self):
        """ Function to place cup on base for transport """
        self.arm.move_commander.clear_path_constraints()

        rospy.sleep(0.5)
        self.arm.gripper.open_gripper()

        # Pre-grasp approach
        eef_pos = cup_pos[:]
        eef_pos[2]+=2*APPROACH_DISTANCE

        grasp_angle = [0,np.deg2rad(90),0]
        pre_transport_grasp = Pose( Point(eef_pos[0], 
            eef_pos[1], 
            eef_pos[2]), 
            euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_transport_grasp)
        self.plan.planning_scene.removeCollisionObject("cup_1")
        rospy.sleep(0.5)

        # Grasp
        self.arm.move_torso_relative(-APPROACH_DISTANCE*2)
        rospy.sleep(0.5)
        # Close Gripper
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Retreat
        self.arm.move_torso_relative(APPROACH_DISTANCE)
        # Attach cup
        self.plan.attach_cup()

        # Init orientation constraint
        self.arm.init_upright_constraint(7,"z")

        # Pre-approach base
        eef_pos = cup_holder[:]
        eef_pos[2] += 5*APPROACH_DISTANCE
        base_pre_approach= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.solve_path_plan(base_pre_approach)
        rospy.sleep(0.5)

        # Remove holder collision object
        self.plan.planning_scene.removeCollisionObject("cup_holder")

        # Place in bracket
        self.arm.move_torso_relative(-APPROACH_DISTANCE*3)
        self.arm.gripper.open_gripper()
        self.plan.planning_scene.removeAttachedObject("gripped_cup")
        self.plan.planning_scene.removeCollisionObject("gripped_cup")

        # Retreat
        self.arm.move_torso_relative(APPROACH_DISTANCE*2)

        # Add back collision object
        self.plan.planning_scene.addCylinder("cup_holder",0.11,0.05,cup_holder[0],cup_holder[1],cup_holder[2])

        # Tuck arm for transport
        self.arm.move_commander.clear_path_constraints()
        #self.arm.tuck_arm()




#####################################################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass







    