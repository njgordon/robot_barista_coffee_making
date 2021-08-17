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

################------------- Global Functions and variables -----------------################    
def euler_to_quat(euler_angles):
    hold_quat_array =tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1] , euler_angles[2])
    quat_object= Quaternion(hold_quat_array [0], hold_quat_array [1], hold_quat_array [2], hold_quat_array [3])
    return quat_object

GRIPPER_ON_CUP = 0.075
TABLE_HEIGHT = 0.89 #1.1 
APPROACH_DISTANCE = 0.05
CUP_TO_MACHINE_OFFSET = 0.22
MACHINE_TO_MARKER_OFFSET = 0.13

# Locations
table_pos = [1, 0, TABLE_HEIGHT/2]
cup_pos = [0.8, -0.15, TABLE_HEIGHT+0.05] #initial pick location
#sugar_pos = [0.9,-0.5,TABLE_HEIGHT+0.05]
grasp_angle = [0, 0 ,0]
machine_location = [1, 0.25, TABLE_HEIGHT+0.05]
cup_holder = [0.22,0,0.42]

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

        # Add objects
        self.planning_scene.addBox("table", 0.6, 2.0, TABLE_HEIGHT-0.01, table_pos[0], table_pos[1], table_pos[2])
        self.planning_scene.addCylinder("cup_1", 0.1, 0.05, cup_pos[0], cup_pos[1], cup_pos[2])
        self.planning_scene.addCylinder("cup_holder",0.11,0.05,cup_holder[0],cup_holder[1],cup_holder[2])
        #self.planning_scene.addCylinder("Sugar",0.1, 0.05, sugar_pos[0],sugar_pos[1],sugar_pos[2])

        # Static machine location for simulation
        #self.planning_scene.addBox("Machine",0.3,0.2,0.2,machine_location[0],machine_location[1],machine_location[2])
        

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

        def get_eef_pos(self):
            pos = [
            self.arm.move_commander.get_current_pose().pose.position.x,
            self.arm.move_commander.get_current_pose().pose.position.y,
            self.arm.move_commander.get_current_pose().pose.position.z]
            return pos