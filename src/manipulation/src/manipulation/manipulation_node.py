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
    #marker_loc = robotManipulation.plan.marker_location()

    #robotManipulation.arm.tuck_arm()

    #####--- Manipulations ---#####
    eef_pos = robotManipulation.pick_cup()
    cup_in_machine_pos = robotManipulation.move_to_machine(eef_pos)

    #eef_pos = robotManipulation.move_to_machine(eef_pos,marker_loc)

    #robotManipulation.open_hatch(eef_pos)
    #robotManipulation.push_button(marker_loc)

    #robotManipulation.remove_cup_from_machine(cup_in_machine_pos)
    #robotManipulation.ready_for_transport()


    #rospy.spin()
    

################------------- Robot manipulation Class -------------################
class RobotManipulation(object):
    def __init__(self):
        self.arm = FetchArm()
        self.plan = RobotPathPlanning()

        #self.arm.move_commander.set_planner_id("RRTConnectkConfigDefault")
        #self.arm.move_commander.set_planner_id("TRRTkConfigDefault")
        self.arm.move_commander.set_planning_time(20)
        self.arm.move_commander.allow_replanning(True)

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

        # Approach
        rospy.loginfo("Approach")
        eef_pos[0] = cup_pos[0]
        #rospy.loginfo(eef_pos[0])
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(1)
        
        # Close Gripper
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.05,0.05,0.03,0,0,'gripper_link')

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

        # Move accross to machine
        eef_pos[1] = machine_location[1]
        #eef_pos[1] = marker_location[1]
        eef_pos[2]+=0.06
        move_across= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 

        self.arm.cartesian_path(eef_pos[1],'y')
        #self.arm.solve_path_plan(move_across)
        rospy.sleep(0.5)

        # move forward
        self.plan.planning_scene.removeCollisionObject("Machine")
        #self.plan.planning_scene.addBox("Machine w/ cup",0.3,0.2,0.2,machine_location[0]+0.2,machine_location[1],machine_location[2])
          
        eef_pos[0] = machine_location[0] - CUP_TO_MACHINE_OFFSET
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(0.5)

        # place cup
        cup_in_machine = eef_pos[:] # copy values
        self.arm.gripper.open_gripper()
        self.plan.planning_scene.removeAttachedObject('gripped_cup')
        self.plan.planning_scene.removeCollisionObject('gripped_cup')
        rospy.sleep(1)

        # move gripper back
        eef_pos[0]-= APPROACH_DISTANCE
        retreat_machine=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle))
        self.arm.move_gripper_to_pose(retreat_machine)   
        rospy.sleep(0.5)

        #self.arm.move_commander.clear_path_constraints()
        # Add machine back
        self.plan.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])
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
        eef_pos[1] = marker_loc[1] - 0.11
        eef_pos[2] = marker_loc[2] + 0.23
        grasp_angle=[np.deg2rad(-45),0,0]
        pre_button_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_button_pose)
        rospy.sleep(1)
        
        self.arm.gripper.open_gripper()
        rospy.sleep(0.5)
        self.arm.gripper.close_gripper(0.027)

    def remove_cup_from_machine(self,cup_in_machine_pos):

        self.arm.init_upright_constraint(7,"x")
        eef_pos = cup_in_machine_pos[:] # copy values

        # Pre-approach
        eef_pos[0]-=APPROACH_DISTANCE
        pre_cup_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_cup_pose)

        # Approach
        eef_pos[0]+=APPROACH_DISTANCE
        self.plan.planning_scene.removeCollisionObject("Machine")
        self.arm.cartesian_path(cup_in_machine_pos[0],"x")     
        rospy.sleep(0.5)

        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.07,0.08,0,0,0,'gripper_link')

        # move gripper back
        eef_pos[0]-=APPROACH_DISTANCE
        self.arm.cartesian_path(eef_pos[0],"x") 

         # Add machine back
        self.plan.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])

        # Pre-grasp: Move accross back to original position
        eef_pos=[cup_pos[0] - APPROACH_DISTANCE, cup_pos[1], cup_pos[2]]
        move_across= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.solve_path_plan(move_across)
        rospy.sleep(0.5)

        # Place back on table
        eef_pos[0]+=APPROACH_DISTANCE
        self.arm.cartesian_path(eef_pos[0],"x")
        rospy.sleep(1)

        # TODO: lower torso slowly for placing cup, with coffee in it!

        self.plan.planning_scene.removeAttachedObject('gripped_cup')
        self.plan.planning_scene.removeCollisionObject('gripped_cup')
       
        # Move back
        eef_pos[0]-=APPROACH_DISTANCE*3
        self.cartesian_path(eef_pos[0],"x")
        self.plan.planning_scene.addCylinder("cup_1", 0.1, 0.05, cup_pos[0], cup_pos[1], cup_pos[2])

    def ready_for_transport(self):
        """ Function to place cup on base for transport """
        self.arm.move_commander.clear_path_constraints()
        self.arm.gripper.open_gripper()

        # Pre-grasp approach
        eef_pos = cup_pos[:]
        eef_pos[2]+=3*APPROACH_DISTANCE
        grasp_angle = [0,np.deg2rad(90),0]
        pre_grasp = Pose( Point(eef_pos[0], 
            eef_pos[1], 
            eef_pos[2]), 
            euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_grasp)
        self.plan.planning_scene.removeCollisionObject("cup_1")
        rospy.sleep(0.5)

        # Grasp
        eef_pos[2]+=APPROACH_DISTANCE
        self.arm.cartesian_path(eef_pos[2],"z")
        rospy.sleep(0.5)
        # Close Gripper
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.05,0.05,0.03,0,0,'gripper_link')
        rospy.sleep(0.5)

        # Retreat
        eef_pos[2]+=2*APPROACH_DISTANCE
        self.arm.cartesian_path(eef_pos[2],"z")

        # Init orientation constraint
        self.arm.init_upright_constraint(7,"z")

        # Pre-approach base
        eef_pos = cup_holder[:]
        eef_pos[2] += 3*APPROACH_DISTANCE
        base_pre_approach= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.solve_path_plan(base_pre_approach)
        rospy.sleep(0.5)

        # Remove holder collision object
        self.plan.planning_scene.removeCollisionObject("cup_holder")

        # Place in bracket
        #eef_pos[2]+=0.5*APPROACH_DISTANCE
        #self.cartesian_path(eef_pos[2],"z")
        torso = self.arm.getJointPosition("torso_lift_joint")
        self.arm.move_torso(torso-0.05)
        self.arm.gripper.open_gripper()
        self.plan.planning_scene.removeAttachedObject("gripped_cup")
        self.plan.planning_scene.removeCollisionObject("gripped_cup")

        # Retreat
        #eef_pos[2] += 2*APPROACH_DISTANCE
        #self.cartesian_path(eef_pos[2],"z")
        self.arm.move_torso(torso+0.05)

        # Add back collision object
        self.plan.planning_scene.addCylinder("cup_holder",0.11,0.05,cup_holder[0],cup_holder[1],cup_holder[2])

        # Tuck arm for transport
        self.arm.tuck_arm()


#####################################################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass







    