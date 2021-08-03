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
# vision imports
from ar_track_alvar_msgs.msg import AlvarMarkers


############################################################################################################
def main():
    rospy.init_node('Manipulation')

    # Create class objects
    robotManipulation = RobotManipulation()
    marker_loc = robotManipulation.plan.marker_location()

    #robotManipulation.arm.tuck_arm()

    #####--- Manipulations ---#####
    #eef_pos = robotManipulation.pick_cup()
    #cup_in_machine_pos = robotManipulation.move_to_machine(eef_pos)



    #eef_pos = robotManipulation.move_to_machine(eef_pos,marker_loc)

    #robotManipulation.open_hatch(eef_pos)
    robotManipulation.push_button(marker_loc)

    #robotManipulation.remove_cup_from_machine(cup_in_machine_pos)
    #robotManipulation.ready_for_transport()


    #rospy.spin()
    

################------------- Global Functions and variables -----------------################    
def euler_to_quat(euler_angles):
    hold_quat_array =tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1] , euler_angles[2])
    quat_object= Quaternion(hold_quat_array [0], hold_quat_array [1], hold_quat_array [2], hold_quat_array [3])
    return quat_object

GRIPPER_ON_CUP = 0.075
TABLE_HEIGHT = 0.89 #1.1 
APPROACH_DISTANCE = 0.05
CUP_TO_GRIPPER_OFFSET = 0.1
CUP_TO_MACHINE_OFFSET = 0.22
MACHINE_TO_MARKER_OFFSET = 0.13

# Locations
table_pos = [1, 0, TABLE_HEIGHT/2]
cup_pos = [0.75, -0.15, TABLE_HEIGHT+0.05] #initial pick location
#sugar_pos = [0.9,-0.5,TABLE_HEIGHT+0.05]
grasp_angle = [0, 0 ,0]
machine_location = [1, 0.25, TABLE_HEIGHT+0.1]
cup_bracket = [0.1,0.1,0.5]

################------------- Robot manipulation Class -------------################
class RobotManipulation(object):
    def __init__(self):
        self.arm = FetchArm()
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

    def init_upright_constraint(self, tolerance_deg, constrained_axis):
        """ Initialise upright constraint with tolerance in degrees for all axes. """

        # Axis to constrain
        if constrained_axis == "x":
            constrained_orientation = euler_to_quat([0,0,0])
        elif constrained_axis == "z":
            constrained_orientation = euler_to_quat([0,np.deg2rad(90),0])
        else:
            return "Error: Invalid axis"

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
        orientation_constraint.orientation = constrained_orientation
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orientation_constraint)
        self.arm.move_commander.set_path_constraints(constraints)
        rospy.loginfo("Upright constraint applied with %s deg tolerance",tolerance_deg)
        #rospy.loginfo(self.arm.move_commander.get_path_constraints())

    def cartesian_path(self, axis_pos, axis):
        waypoints = []
        segments = 5
        wpose = self.arm.move_commander.get_current_pose().pose
        #rospy.loginfo(wpose.position.x)

        if axis =="x":
            dif = axis_pos-wpose.position.x
        elif axis =="y":
            dif = axis_pos-wpose.position.y
        elif axis =="z":
            dif = axis_pos-wpose.position.z
        else:
            return "Error: Invalid axis"

        for i in range(0,segments):
            if axis =="x":
                wpose.position.x += dif/segments  # Second move forward/backwards in axis direction
            elif axis =="y":
                wpose.position.y += dif/segments  # Second move forward/backwards in axis direction
            elif axis =="z":
               wpose.position.z += dif/segments  # Second move forward/backwards in axis direction
            
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:

        (plan, fraction) = self.arm.move_commander.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        rospy.loginfo("Waypoint fraction: %s",fraction)
        
        # Set max path speed
        plan_retimed = self.arm.move_commander.retime_trajectory(self.arm.move_commander.get_current_state(),plan,self.arm.MAX_VELOCITY_SCALING_FACTOR)

        self.arm.move_commander.execute(plan_retimed,wait=True)


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
        eef_pos=[cup_pos[0] - CUP_TO_GRIPPER_OFFSET - APPROACH_DISTANCE, cup_pos[1],cup_pos[2]]
        rospy.loginfo("Pre-grasp")
        cup_pre_grasp_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(cup_pre_grasp_pose)

        # Make sure gripper is open
        self.arm.gripper.open_gripper()
      
        self.plan.planning_scene.removeCollisionObject("cup_1")

        # Approach
        rospy.loginfo("Approach")
        eef_pos[0] = cup_pos[0] - CUP_TO_GRIPPER_OFFSET
        self.cartesian_path(eef_pos[0],"x")
        rospy.sleep(1)
        
        # Close Gripper
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.07,0.1,0,0,0,'gripper_link')

        # Lift up
        #torso = self.arm.getJointPosition("torso_lift_joint")
        #self.arm.move_torso(torso+0.06)
        eef_pos[2]+=0.05
        lift = Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(lift)

        # Move back
        eef_pos[0]-=0.2
        self.cartesian_path(eef_pos[0],"x")
        rospy.sleep(0.5)

        return eef_pos
        
    def move_to_machine(self, eef_pos):
        """ Function to move cup to machine. """
        rospy.loginfo("Move to machine")
        self.init_upright_constraint(10,"x")

        # Move accross to machine
        eef_pos[1] = machine_location[1]
        #eef_pos[1] = marker_location[1]
        eef_pos[2]+=0.06
        move_across= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 

        self.arm.solve_path_plan(move_across)
        rospy.sleep(0.5)

        # move forward
        self.plan.planning_scene.removeCollisionObject("Machine")
        #self.plan.planning_scene.addBox("Machine w/ cup",0.3,0.2,0.2,machine_location[0]+0.2,machine_location[1],machine_location[2])
          
        eef_pos[0] = machine_location[0] - CUP_TO_MACHINE_OFFSET
        self.cartesian_path(eef_pos[0],"x")
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

        self.init_upright_constraint(7,"x")
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
        self.cartesian_path(cup_in_machine_pos[0],"x")     
        rospy.sleep(0.5)

        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        rospy.sleep(0.5)

        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.07,0.08,0,0,0,'gripper_link')

        # move gripper back
        eef_pos[0]-=APPROACH_DISTANCE
        self.cartesian_path(eef_pos[0],"x") 

         # Add machine back
        self.plan.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])

        # Pre-grasp: Move accross back to original position
        eef_pos=[cup_pos[0] - CUP_TO_GRIPPER_OFFSET - APPROACH_DISTANCE, cup_pos[1], cup_pos[2]]
        move_across= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.solve_path_plan(move_across)
        rospy.sleep(0.5)

        # Place back on table
        eef_pos[0]+=APPROACH_DISTANCE
        self.cartesian_path(eef_pos[0],"x")
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
        eef_pos[2]+=2*APPROACH_DISTANCE
        grasp_angle = [0,np.deg2rad(90),0]
        pre_grasp = Pose( Point(eef_pos[0], 
            eef_pos[1], 
            eef_pos[2]), 
            euler_to_quat(grasp_angle)) 
        self.arm.move_gripper_to_pose(pre_grasp)
        rospy.sleep(1)

        # Grasp
        self.plan.planning_scene.removeCollisionObject("cup_1")
        #eef_pos[2]-=APPROACH_DISTANCE
        #self.cartesian_path(eef_pos[2],"z")

        # Close Gripper
        self.arm.gripper.close_gripper(GRIPPER_ON_CUP)
        # Attach cup
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.07,0.08,0,0,0,'gripper_link')
        rospy.sleep(1)

        # Retreat
        eef_pos[2]+=2*APPROACH_DISTANCE
        self.cartesian_path(eef_pos[2],"z")

        # Init orientation constraint
        self.init_upright_constraint(7,"z")

        # Pre-approach base
        eef_pos = cup_bracket
        base_pre_approach= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                euler_to_quat(grasp_angle)) 
        self.arm.solve_path_plan(base_pre_approach)
        rospy.sleep(0.5)

        # Remove base collision

        # Place in bracket
        self.arm.gripper.open_gripper()


################------------- Robot planning Class -----------------################
class RobotPathPlanning(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.head = FetchHead()

        # Create the scene
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("table")
        self.planning_scene.removeCollisionObject("cup_1")
        #self.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])

        # Add objects
        self.planning_scene.addBox("table", 0.6, 2.0, TABLE_HEIGHT, table_pos[0], table_pos[1], table_pos[2])
        self.planning_scene.addCylinder("cup_1", 0.1, 0.05, cup_pos[0], cup_pos[1], cup_pos[2])
        #self.planning_scene.addCylinder("Sugar",0.1, 0.05, sugar_pos[0],sugar_pos[1],sugar_pos[2])
        # TODO: add cup brakcet on base

    def marker_location(self):
        #rate = rospy.Rate(0.5)
        flag=0

        while not rospy.is_shutdown():
            try:
                marker_transfrom = self.tfBuffer.lookup_transform('base_link','ar_marker_0',rospy.Time())
                rospy.loginfo(marker_transfrom)
                break
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:       
                # Tilt head to find marker
                if flag:
                    self.head.turn_head({"direction":"left","angle_deg":"10"})
                    flag=0
                else:
                    self.head.turn_head({"direction":"right","angle_deg":"10"})
                    flag=1

                #rate.sleep()
                rospy.sleep(3)
                continue
        loc =[0]*3
        loc[0] = marker_transfrom.transform.translation.x
        loc[1] = marker_transfrom.transform.translation.y
        loc[2] = marker_transfrom.transform.translation.z

        # Add machine object
        global machine_location
        machine_location[0]=loc[0]
        machine_location[1]=loc[1]
        self.planning_scene.addBox("Machine",0.27,0.2,0.3,machine_location[0]+MACHINE_TO_MARKER_OFFSET,machine_location[1],machine_location[2])

        return loc

#####################################################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass







    