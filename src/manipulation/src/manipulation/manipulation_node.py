#!/usr/bin/env python

# standard ROS imports
import sys
import time
import math
import rospy
import copy
from std_msgs.msg import ColorRGBA, Float32
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

############################################################################################################
def main():
    rospy.init_node('Manipulation')
    rospy.loginfo('Start Manipulation Node')

    # Create class objects
    robotManipulation = RobotManipulation()

    robotManipulation.arm.tuck_arm()

    # Movemments
    eef_pos = robotManipulation.pick_cup()

    eef_pos = robotManipulation.move_to_machine(eef_pos)

    robotManipulation.push_button(eef_pos)

    #robotManipulation.remove_cup_from_machine()
    #robotManipulation.ready_for_transport()
    #robotManipulation.open_hatch(eef_pos)

    

################------------- Global Functions and variables -----------------################

def euler_to_quat(euler_angles):
    hold_quat_array =tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1] , euler_angles[2])
    quat_object= Quaternion(hold_quat_array [0], hold_quat_array [1], hold_quat_array [2], hold_quat_array [3])
    return quat_object

gripper_on_cup = 0.075
table_height = 0.88 #1.1 

# Locations
table_pos = [1, 0, table_height/2]
cup_pos = [0.8, 0, table_height+0.05] #initial pick location
sugar_pos = [0.9,-0.5,table_height+0.05]
grasp_angle = [0, 0 ,0]
machine_location = [1, 0.25, table_height+0.05]
 
################------------- Robot manipulation Class -------------################
class RobotManipulation(object):
    def __init__(self):
        self.arm = FetchArm()
        self.head = FetchHead()
        self.plan = RobotPathPlanning()

        self.arm.move_commander.set_planner_id("RRTConnectkConfigDefault")
        #self.arm.move_commander.set_planner_id("TRRTkConfigDefault")
        self.arm.move_commander.set_planning_time(10)
        self.arm.move_commander.allow_replanning(True)

    def get_eef_pos(self):
        pos = [
            self.arm.move_commander.get_current_pose().pose.position.x,
            self.arm.move_commander.get_current_pose().pose.position.y,
            self.arm.move_commander.get_current_pose().pose.position.z]
        return pos

    def init_upright_constraint(self,tolerance_deg):
        """ Initialise upright constraint with tolerance in degrees for all axes. """
        self.arm.move_commander.clear_pose_targets()
        self.arm.move_commander.clear_path_constraints()
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        constraints = moveit_msgs.msg.Constraints()

        tolerance = np.deg2rad(tolerance_deg)
        start_pose = self.arm.move_commander.get_current_pose("gripper_link")

        constraints.name = "upright"
        self.arm.move_commander.set_pose_reference_frame("base_link")
        orientation_constraint.header = start_pose.header
        orientation_constraint.link_name = "gripper_link"
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.orientation = euler_to_quat([0,0,0])
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orientation_constraint)
        self.arm.move_commander.set_path_constraints(constraints)
        rospy.loginfo("Upright constraint applied with %s deg tolerance",tolerance_deg)
        #rospy.loginfo(self.arm.move_commander.get_path_constraints())

    def cartesian_path(self, x_pos):
        waypoints = []
        segments = 5
        wpose = self.arm.move_commander.get_current_pose().pose
        #rospy.loginfo(wpose.position.x)

        dif = x_pos-wpose.position.x

        for i in range(0,segments):
            wpose.position.x += dif/segments  # Second move forward/backwards in (x)
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:

        (plan, fraction) = self.arm.move_commander.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        rospy.loginfo("Waypoint fraction: %s",fraction)
        self.arm.move_commander.execute(plan,wait=True)


    ################------------- Manipulation functions -------------################
    def pick_cup(self):
        """ Function to pick up a cup. """
        rospy.loginfo("Pick Cup")
        #self.arm.move_joint("shoulder_pan_joint", np.deg2rad(-45))    # Move arm so that it doesn't obstruct the camera view CONVERT TO RADIANS

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

        approach_distance = 0.05
        z_offset = 0.0

        # Pre grasp approach
        eef_pos=[cup_pos[0] - 0.1 - approach_distance, cup_pos[1],cup_pos[2]+z_offset]
        rospy.loginfo("Pre-grasp")
        cup_pre_grasp_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(cup_pre_grasp_pose)
        rospy.loginfo(eef_pos)

        # Make sure gripper is open
        self.arm.gripper.open_gripper()

        # Planning scene updates
        #self.planning_scene.removeCollisionObject("table")
        #self.planning_scene.addBox("table", 0.7, 2.0, (table_pos[2]*2), table_pos[0], table_pos[1], table_pos[2])
      
        self.plan.planning_scene.removeCollisionObject("cup_1")

        # Approach
        rospy.loginfo("Approach")
        eef_pos[0] = cup_pos[0] - 0.1
        self.cartesian_path(eef_pos[0])
        rospy.sleep(1)
        
        # Close Gripper
        self.arm.gripper.close_gripper(gripper_on_cup)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.07,0.08,0,0,0,'gripper_link')

        # Lift up
        torso = self.arm.getJointPosition("torso_lift_joint")
        self.arm.move_torso(torso+0.06)

        # Move back
        self.init_upright_constraint(10)
        eef_pos[0]-=0.1
        self.cartesian_path(eef_pos[0])
        rospy.sleep(0.5)

        return eef_pos
        
    def move_to_machine(self, eef_pos):
        """ Function to move cup to machine. """
        
        rospy.loginfo("Move to machine")
        #eef_pos = self.get_eef_pos()
                
        # Move accross to machine
        eef_pos[1] = machine_location[1]
        eef_pos[2]+=0.05
        move_across= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 

        plan = self.arm.solve_path_plan(move_across)
        self.arm.move_commander.execute(plan)
        rospy.sleep(0.5)

        # move forward
        self.plan.planning_scene.removeCollisionObject("Machine")
        #self.plan.planning_scene.addBox("Machine w/ cup",0.3,0.2,0.2,machine_location[0]+0.2,machine_location[1],machine_location[2])
          
        eef_pos[0] = machine_location[0] - 0.3
        self.cartesian_path(eef_pos[0])
        rospy.sleep(0.5)

        # place cup
        self.arm.gripper.open_gripper()
        self.plan.planning_scene.removeAttachedObject('gripped_cup')
        self.plan.planning_scene.removeCollisionObject('gripped_cup')
        rospy.sleep(1)

        # move gripper back
        eef_pos[0]-=0.05
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
        return eef_pos

    def open_hatch(self):
        """ Function to open hatch. """
        eef_pos = self.get_eef_pos()

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

    def push_button(self,eef_pos):
        """ Function to push button. """

        #eef_pos = self.get_eef_pos()
        # Pre-button
        eef_pos[0] = machine_location[0] - 0.1
        eef_pos[1]-=0.2
        eef_pos[2]+=0.2
        grasp_angle=[np.deg2rad(-45),0,0]
        pre_button_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_button_pose)
        rospy.sleep(1)
        
        # Push button
        eef_pos[1]+=0.05
        eef_pos[2]-=0.05
        button_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(button_pose)
        
        #Retreat
        self.arm.move_gripper_to_pose(pre_button_pose)

        """
        # Push button
        wrist_flex = self.arm.getJointPosition('wrist_flex_joint')
        roll = np.deg2rad(10)
        wrist_flex-=roll
        self.arm.move_joint('wrist_flex_joint',wrist_flex)

        # Release
        self.arm.move_joint('wrist_flex_joint',wrist_flex+roll)
        """
    def remove_cup_from_machine(self):

        self.init_upright_constraint(7)

        # TODO: Reduce velocity

        # Pre-approach
        eef_pos = [0.65, 0.5, 1.17]
        pre_cup_pose = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_cup_pose)

        # Approach
        self.plan.planning_scene.removeCollisionObject("Machine")
        eef_pos[0]+=0.25
        pick_cup=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pick_cup)       
        rospy.sleep(0.5)

        self.arm.gripper.close_gripper(gripper_on_cup)
        rospy.sleep(1)

        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.07,0.08,0,0,0,'gripper_link')

        # move gripper back
        eef_pos[0]-=0.2
        retreat_machine=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle))
        self.arm.move_gripper_to_pose(retreat_machine)   

         # Add machine back
        self.plan.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])

        # Move accross back to original position
        eef_pos[1]-=0.5
        move_across= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 

        plan = self.arm.solve_path_plan(move_across)
        self.arm.move_commander.execute(plan)

        rospy.sleep(0.5)

        # Place back on table
        eef_pos[0] = cup_pos[0] - 0.05
        eef_pos[1] = cup_pos[1]
        eef_pos[2] = cup_pos[2] + 0.05
        cup_grasp_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(cup_grasp_pose)
        #self.cartesian_path(eef_pos[0])

        # TODO: lower torso slowly for placing cup, with coffee in it!

        self.plan.planning_scene.removeAttachedObject('gripped_cup')
        self.plan.planning_scene.removeCollisionObject('gripped_cup')
       
        # Move back
        eef_pos[0]-=0.2
        self.cartesian_path(eef_pos[0])
        self.plan.planning_scene.addCylinder("cup_1", 0.1, 0.05, cup_pos[0], cup_pos[1], cup_pos[2])

    def ready_for_transport(self):
        eef_pos = self.get_eef_pos()
        self.init_upright_constraint(7)


        eef_pos[0]-=0.2
        eef_pos[1]-=0.4
        #ready = Pose( Point(0.4, -0.1, 0.9), euler_to_quat(grasp_angle))
        ready = Pose( Point(eef_pos[0],eef_pos[1],eef_pos[2]), euler_to_quat(grasp_angle))
        plan = self.arm.solve_path_plan(ready)
        self.arm.move_commander.execute(plan)

        # joint_goal = self.arm.move_commander.get_current_joint_values()
        # joint_goal[1] = np.deg2rad(54)
        # joint_goal[2] = np.deg2rad(78)
        # joint_goal[3] = np.deg2rad(-32)
        # joint_goal[4] = np.deg2rad(120)
        # joint_goal[5] = np.deg2rad(86)
        # joint_goal[6] = np.deg2rad(-104)
        
        # self.arm.move_group.moveToJointPosition(self.arm.joints_name,joint_goal)

        # joints = self.arm.getJointsPosition(self.arm.joints_name)
        # rospy.loginfo(joints)
        # rospy.loginfo(joint_goal)
        # self.arm.move_joint("shoulder_pan_joint",np.deg2rad(90))

################------------- Robot planning Class -----------------################
class RobotPathPlanning(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Create the scene
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("table")
        self.planning_scene.removeCollisionObject("cup_1")
        self.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])

        # Add objects
        self.planning_scene.addBox("table", 0.6, 2.0, table_height, table_pos[0], table_pos[1], table_pos[2])
        self.planning_scene.addCylinder("cup_1", 0.1, 0.05, cup_pos[0], cup_pos[1], cup_pos[2])
        self.planning_scene.addCylinder("Sugar",0.1, 0.05, sugar_pos[0],sugar_pos[1],sugar_pos[2])

#####################################################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass







    