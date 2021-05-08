#!/usr/bin/env python

# standard ROS imports
import sys
import time
import math
import rospy
import copy
from std_msgs.msg import ColorRGBA, Float32

# move_arm imports
import geometry_msgs.msg 
from moveit_msgs.msg import MoveItErrorCodes, Grasp, PlaceLocation, RobotState, Constraints, OrientationConstraint
from moveit_python import *
# from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# move_head imports
from control_msgs.msg import PointHeadAction, PointHeadGoal
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib # also used for move_base to pose
import tf2_ros
# local modules for Fetch primitives
from move_arm.fetch_move_arm import *
from move_head.fetch_move_head import *

import tf2_ros
import tf
import tf_conversions
from tf import transformations
import numpy as np

# Position and geometry global variables
gripper_on_cup = 0.075
table_height = 0.88
table_pos = [1.0, 0, table_height/2]
cup_pos = [0.8, 0, table_height+0.05] #initial pick location
grasp_angle = [0, 0 ,0]
machine_location = [0.9, 0.5, table_height+0.05]

############################################################################################################
def main():
    rospy.init_node('Manipulation')
    rospy.loginfo('Start Manipulation Node')

    # Create class objects
    robotManipulation = RobotManipulation()
    robotPathPlanning = RobotPathPlanning()
    gripper = FetchGripper()

    #robotManipulation.arm.tuck_arm()

    # Movemments
    #eef_pos = robotManipulation.pick_cup()
    #eef_pos = robotManipulation.move_to_machine(eef_pos)
    robotManipulation.arm.remove_constraints()
    #robotManipulation.open_hatch(eef_pos)

    #grip_loc = robotManipulation.curr_gripper_location()

############################################################################################################


def make_quat_object(euler_angles):
    hold_quat_array =tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1] , euler_angles[2])
    quat_object= Quaternion(hold_quat_array [0], hold_quat_array [1], hold_quat_array [2], hold_quat_array [3])
    return quat_object

################------------- Robot planning Class -------------################
class RobotPathPlanning(object):
    def __init__(self):
        self.upright_constraints = Constraints()

        # Create the scene
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("table")
        self.planning_scene.removeCollisionObject("cup_1")
        self.planning_scene.removeAttachedObject('gripped_cup')
        self.planning_scene.removeCollisionObject("gripped_cup")
        
        # Add objects
        self.planning_scene.addBox("table", 0.6, 2.0, table_height, table_pos[0], table_pos[1], table_pos[2])
        #self.planning_scene.addBox("table", 0.6, 1.0, 0.15 + (table_pos[2]*2), table_pos[0], table_pos[1], table_pos[2])
        self.planning_scene.addCylinder("cup_1", 0.1, 0.05, cup_pos[0], cup_pos[1], cup_pos[2])
        
    def init_upright_constraint(self,pose):
        self.upright_constraints.name = "upright"
        orientation_constraint = OrientationConstraint()
        #orientation_constraint.header = pose.header
        orientation_constraint.link_name = 'gripper_link'
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.4
        orientation_constraint.absolute_y_axis_tolerance = 0.4
        orientation_constraint.absolute_z_axis_tolerance = 0.4
        orientation_constraint.weight = 1
        self.upright_constraints.orientation_constraints.append(orientation_constraint)
    
    def curr_gripper_location(self):
        location = [0]*3
        orientation = [0]*4
        rate = rospy.Rate(10.0)
        # TODO: create loop to wait for transform
        try:
            gripper_transfrom = self.tfBuffer.lookup_transform('gripper_link','base_link',rospy.Time())
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            rate.sleep()

        rospy.loginfo(gripper_transfrom)
        location[0] = gripper_transfrom.transform.translation.x
        location[1] = gripper_transfrom.transform.translation.y
        location[2] = gripper_transfrom.transform.translation.z

        orientation[0] = gripper_transfrom.transform.rotation.x
        orientation[1] = gripper_transfrom.transform.rotation.y
        orientation[2] = gripper_transfrom.transform.rotation.z
        orientation[3] = gripper_transfrom.transform.rotation.w
        return location, orientation

################------------- Robot manipulation Class -------------################
class RobotManipulation(object):
    def __init__(self):
        self.arm = FetchArm()
        self.head = FetchHead()
        self.plan = RobotPathPlanning()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # Function to pick up a cup
    def pick_cup(self):
        
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

        #self.arm.move_joint("shoulder_pan_joint", np.deg2rad(0))

        rospy.loginfo("cup 1 (x,y,z) = %s,%s,%s", cup_pos[0], cup_pos[1], cup_pos[2])

        
        approach_distance = 0.20
        lift_distance = 0.1
        objects_height = 0.20
        z_offset = 0.05

        # Add objects
        #self.planning_scene.addBox("table", 0.6, 1.0, 0.15 + (table_pos[2]*2), table_pos[0], table_pos[1], table_pos[2])
        #self.planning_scene.addBox("cup_1", 0.02, 0.02, (cup_pos[2]- (table_pos[2]*2))*2, cup_pos[0], cup_pos[1], cup_pos[2])
        #self.planning_scene.addBox("cup_2", 0.05, 0.05, (cup2_pos[2]- (table_pos[2]*2))*2, cup2_pos[0], cup2_pos[1], cup2_pos[2])
        rospy.sleep(1.5)

        # Make sure gripper is open
        self.arm.gripper.open_gripper()

        #self.arm.preGrasp_pose()
        eef_pos=[cup_pos[0] - approach_distance,cup_pos[1],cup_pos[2]+z_offset]
        # Pre grasp approach
        cup_pre_grasp_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(cup_pre_grasp_pose)
        rospy.loginfo("Pre-grasp")
        rospy.sleep(1)

        # Planning scene updates
        #self.planning_scene.removeCollisionObject("table")
        #self.planning_scene.addBox("table", 0.7, 2.0, (table_pos[2]*2), table_pos[0], table_pos[1], table_pos[2])
        self.plan.planning_scene.removeCollisionObject("cup_1")

        # Approach
        rospy.loginfo("Approach")
        eef_pos[0] = cup_pos[0]
        eef_pos[1] = cup_pos[1]
        eef_pos[2] = cup_pos[2]
        cup_grasp_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(cup_grasp_pose)
        rospy.sleep(1)
        
        # Close Gripper
        self.arm.gripper.close_gripper(gripper_on_cup)
        rospy.sleep(0.5)

        # Attach objects
        self.plan.planning_scene.attachBox('gripped_cup',0.05,0.07,0.08,0,0,0,'gripper_link')
        #self.planning_scene.removeCollisionObject("Avoid Objects")
        #self.planning_scene.addBox("avoid_objects", 0.05, cup_pos-cup2_pos, objects_height, cup_pos[1], cup2_pos[1], cup2_pos[2])

        # Lift up and back
        eef_pos[0]-=0.1
        eef_pos[2]+=0.1
        move_up_pose = Pose( Point(eef_pos[0], 
                        eef_pos[1], 
                        eef_pos[2]), 
                        make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(move_up_pose)

        return eef_pos
      
        
    def move_to_machine(self,eef_pos):
        rospy.loginfo("Move to machine")

        self.plan.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])

        # Move accross to machine
        eef_pos[1]+=0.5
        move_across= Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                make_quat_object(grasp_angle)) 

        self.plan.init_upright_constraint(move_across)
        self.arm.move_gripper_to_pose(move_across) # constrain gripper facing up
        rospy.sleep(1)

        # move down
        eef_pos[2]-=0.05
        move_down=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(move_down)
        
        # move forward
        self.plan.planning_scene.removeCollisionObject("Machine")
        eef_pos[0]+=0.1
        place_cup=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(place_cup)       
        rospy.sleep(1)

        # place cup
        self.arm.gripper.open_gripper()
        self.plan.planning_scene.removeAttachedObject('gripped_cup')
        self.plan.planning_scene.removeCollisionObject('gripped_cup')
        rospy.sleep(2)

        # move gripper back
        eef_pos[0]-=0.2
        retreat_machine=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(retreat_machine)   
        rospy.sleep(1)

        return eef_pos

    def open_hatch(self,eef_pos):

        # Tilt gripper and move to hatch
        grasp_angle = [np.deg2rad(90),0,0]
        eef_pos[2]+=0.1
        tilt_grip=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(tilt_grip)   

        # Lift up and forward to open hatch
        eef_pos[0]+=0.05
        eef_pos[2]+=0.05
        hatch_pose=Pose( Point(eef_pos[0], 
                eef_pos[1], 
                eef_pos[2]), 
                make_quat_object(grasp_angle)) 
        self.arm.move_gripper_to_pose(hatch_pose)   

if __name__ == '__main__':
    main()







    