#!/usr/bin/env python

# standard ROS imports
import sys
import time
import math
#from copy import copy #move_head by joint angle
import rospy
from std_msgs.msg import ColorRGBA, Float32

# move_arm imports
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, Grasp, PlaceLocation, RobotState, Constraints, OrientationConstraint
from moveit_python import *
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Joy

import actionlib
import numpy as np
#from scipy.spatial.transform import Rotation
import tf2_ros
import tf
import tf_conversions
from tf import transformations
from tuck_arm import TuckThread
from manipulation.srv import *

import numpy as np

#from tf.transforms import *
class FetchArm(object):

    MAX_TORSO = 0.4
    MIN_TORSO = 0.15
    tuck_arm_pos = [0.4, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    MAX_VELOCITY_SCALING_FACTOR = 0.3

    # init adapted from Fetch manual
    def __init__(self):
        
        # TF joint names
        self.joints_name = ["torso_lift_joint", 
                            "shoulder_pan_joint",
                            "shoulder_lift_joint", 
                            "upperarm_roll_joint",
                            "elbow_flex_joint", 
                            "forearm_roll_joint",
                            "wrist_flex_joint", 
                            "wrist_roll_joint"]

        # Define ground plane and base edges to avoid collision
        # This creates objects in the planning scene that mimic the ground/base
        # If these were not in place gripper could hit the ground/base
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("ground_keepout")
        self.planning_scene.removeCollisionObject("base_keepout")
        self.planning_scene.addBox("ground_keepout", 1.5, 1.5, 0.02, 0.0, 0.0, -0.012)
        self.planning_scene.addBox("base_keepout", 0.2, 0.5, 0.001, 0.15, 0.0, 0.368)

        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")
        self.move_commander = moveit_commander.MoveGroupCommander("arm")
        self.pick_place = PickPlaceInterface("arm", "gripper")

        # create a gripper object
        self.gripper = FetchGripper()

        self.pressed = False
        self.pressed_last = None
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

        rospy.loginfo("Fetch move arm initialised")

    def joy_callback(self, msg):
        if msg.buttons[self.deadman] > 0:
            self.stop_arm()

    def getJointsPosition(self, jointNames):
        rospy.wait_for_service('return_joint_states')
        try:
            getJoints = rospy.ServiceProxy('return_joint_states', ReturnJointStates)
            jointsStates = getJoints(jointNames)
            return jointsStates.position
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def getJointPosition(self, jointName):
        jointsPosition = self.getJointsPosition(self.joints_name)
        ind = self.joints_name.index(jointName)
        return jointsPosition[ind]
    
    def move_joint(self, jointName, position):
        # Get current joints position
        joint_goal = self.getJointsPosition(self.joints_name)
        
        joint_goal = np.asarray(joint_goal) # Convert from tuple to array
        ind = self.joints_name.index(jointName) # Get joint index
        joint_goal[ind] = position          # Move torso only

        # Plans the joints in self.joints_name to angles in pose
        self.move_group.moveToJointPosition(self.joints_name, joint_goal, wait=False, max_velocity_scaling_factor=self.MAX_VELOCITY_SCALING_FACTOR)
        
        # Since we passed in wait=False above we need to wait here
        self.move_group.get_move_action().wait_for_result()
        result = self.move_group.get_move_action().get_result()
        # TODO refactor result logging from all primitives
        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Joint move successful!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Joint goal in state: %s",
                            self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

        self.stop_arm()

        return "success"

    # Primitive stop_arm - halts all movement by cancelling goals
    def stop_arm(self, parameters={}):
        self.move_group.get_move_action().cancel_all_goals()

    def remove_constraints(self):
        self.move_commander.clear_path_constraints()

    def ready_pose(self):
        self.move_torso(self.MAX_TORSO)
        rospy.sleep(1)
        #self.ready_arm_pos[0] = 0.2

        self.move_group.moveToJointPosition(self.joints_name, self.ready_arm_pos, wait=True, max_velocity_scaling_factor=self.MAX_VELOCITY_SCALING_FACTOR)
        
        # Since we passed in wait=False above we need to wait here
        self.move_group.get_move_action().wait_for_result()
        result = self.move_group.get_move_action().get_result()

        rospy.sleep(1)

        # TODO refactor result logging from all primitives
        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Ready pose successfull")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                    self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")      

    def preGrasp_pose(self):
        #self.move_torso(self.MAX_TORSO)
        #rospy.sleep(1)
        #self.ready_arm_pos[0] = 0.2

        self.move_group.moveToJointPosition(self.joints_name, self.pregrasp_arm_pos, wait=True, max_velocity_scaling_factor=self.MAX_VELOCITY_SCALING_FACTOR)
        
        # Since we passed in wait=False above we need to wait here
        self.move_group.get_move_action().wait_for_result()
        result = self.move_group.get_move_action().get_result()

        rospy.sleep(1)

        # TODO refactor result logging from all primitives
        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Pre-grasp Pose")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                    self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")        

    # Primitive: tuck_arm - reset arm to idle pose
    def tuck_arm(self, parameters={}):
        
        # Check is fetch is already in tuck pose
        jointsPos = self.getJointsPosition(self.joints_name)
        jointsPos = np.asarray(jointsPos) # Convert from tuple to array
        if (jointsPos == self.tuck_arm_pos).all():
            rospy.loginfo("Fetch arm is already tucked")
            return

        # Move torso up
        self.move_torso(self.MAX_TORSO)

        while True:
            # Plans the joints in joints_name to angles in pose
            rospy.sleep(1)
            self.tuck_arm_pos[0] = self.MAX_TORSO
            self.move_group.moveToJointPosition(self.joints_name, self.tuck_arm_pos, wait=False, max_velocity_scaling_factor=self.MAX_VELOCITY_SCALING_FACTOR)

            # Since we passed in wait=False above we need to wait here
            self.move_group.get_move_action().wait_for_result()
            result = self.move_group.get_move_action().get_result()

            # TODO refactor result logging from all primitives
            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Tucked arm successfully")
                    break
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                        self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.") 

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        self.move_group.get_move_action().cancel_all_goals()

        rospy.sleep(1)
        # Move torso down
        self.move_torso(self.MIN_TORSO)
        rospy.sleep(1)


    # Option: to reverse order of animation
    def reverse_order_if_param_set(self, pose_list, parameters):
        try:
            reverse = parameters["reverse"]
        except (ValueError, KeyError) as e:
            reverse = Falseto
        if reverse==True:
            pose_list.reverse()

    # Primitive: wave - 'Wave' the fetch gripper
    # Adapted from Fetch manual tutorial
    def wave(self, parameters={}):
        # This is the wrist link not the gripper itself
        gripper_frame = 'wrist_roll_link'
        # Position and rotation of two "wave end poses"
        gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
                            Quaternion(0.173, -0.693, -0.242, 0.657)),
                        Pose(Point(0.047, 0.545, 1.822),
                            Quaternion(-0.274, -0.701, 0.173, 0.635))]

        # Construct a "pose_stamped" message as required by moveToPose
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'

        # TODO refactor setting params to default
        # repeats primitive for the number specificed by "repeat" param
        try:
            repeat = int(parameters["repeat"])
        except (ValueError, KeyError) as e:
            repeat = 1

        self.reverse_order_if_param_set(gripper_poses, parameters)

        for x in range(repeat):
            for pose in gripper_poses:
                if rospy.is_shutdown():
                    break

                # Finish building the Pose_stamped message
                # If the message stamp is not current it could be ignored
                gripper_pose_stamped.header.stamp = rospy.Time.now()
                # Set the message pose
                gripper_pose_stamped.pose = pose

                # Move gripper frame to the pose specified
                self.move_group.moveToPose(gripper_pose_stamped, gripper_frame, max_velocity_scaling_factor=self.MAX_VELOCITY_SCALING_FACTOR)
                result = self.move_group.get_move_action().get_result()

                # TODO refactor result logging from all primitives
                if result:
                    # Checking the MoveItErrorCode
                    if result.error_code.val == MoveItErrorCodes.SUCCESS:
                        rospy.loginfo("Wave #%s successful!", x+1)
                    else:
                        # If you get to this point please search for:
                        # moveit_msgs/MoveItErrorCodes.msg
                        rospy.logerr("Arm goal in state: %s",
                            self.move_group.get_move_action().get_state())
                else:
                    rospy.logerr("MoveIt! failure no result returned.")

        return "success"

    # Primitive: move torso - fetch moves torso up and down
    def move_torso(self, torso_pose):
        # List of sequence of poses for the single "torso_lift_joint"
        # Range of joint is 0.0-0.4
        rospy.sleep(1)

        if torso_pose > self.MAX_TORSO:
            torso_pose = self.MAX_TORSO
        if torso_pose < self.MIN_TORSO:
            torso_pose = self.MIN_TORSO

        # Get current joints position
        joint_goal = self.getJointsPosition(self.joints_name)
        
        joint_goal = np.asarray(joint_goal) # Convert from tuple to array
        joint_goal[0] = torso_pose          # Move torso only

        # Plans the joints in self.joints_name to angles in pose
        self.move_group.moveToJointPosition(self.joints_name, joint_goal, wait=False, max_velocity_scaling_factor=self.MAX_VELOCITY_SCALING_FACTOR)
        
        # Since we passed in wait=False above we need to wait here
        self.move_group.get_move_action().wait_for_result()
        result = self.move_group.get_move_action().get_result()
        # TODO refactor result logging from all primitives
        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Torso move successful!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Torso goal in state: %s",
                            self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

        self.stop_arm()

        return "success"

    def move_gripper_to_pose(self, gripper_pose):
        self.__move_hand([gripper_pose, ])

    # Private helper for moving hand
    def __move_hand(self, gripper_poses):
        # gripper link origin is in air mid-way between two gripper fingers
        # i.e. ideal location to pick something
        # x = away from hand(on axis of joint)
        # y = towards right finger
        # z =  pointing out of top of hand
        gripper_frame = 'gripper_link'

        # Construct a "pose_stamped" message as required by moveToPose
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'

        # Pose contains: Point position (x,y,z), Quaternion orientation (x,y,z,w)
        for pose in gripper_poses:
            while True:
                # Finish building the Pose_stamped message
                # If the message stamp is not current it could be ignored
                gripper_pose_stamped.header.stamp = rospy.Time.now()
                # Set the message pose
                gripper_pose_stamped.pose = pose
                # Move gripper frame to the pose specified
                rospy.loginfo("moveToPose")
                self.move_group.moveToPose(gripper_pose_stamped, gripper_frame, max_velocity_scaling_factor=self.MAX_VELOCITY_SCALING_FACTOR)
                self.move_group.get_move_action().wait_for_result()
                result = self.move_group.get_move_action().get_result()

                rospy.loginfo("moveToPose successful")
                # TODO refactor result logging from all primitives
                if result:
                    # Checking the MoveItErrorCode
                    if result.error_code.val == MoveItErrorCodes.SUCCESS:
                        rospy.loginfo("Move hand successful!")
                        break
                    else:
                        # If you get to this point please search for:
                        # moveit_msgs/MoveItErrorCodes.msg
                        rospy.logerr("Arm goal in error val: %s",
                            result.error_code.val)
                        rospy.logerr("Arm goal in state: %s",
                            self.move_group.get_move_action().get_state())
                else:
                    rospy.logerr("MoveIt! failure no result returned.")

        return "success"
        rospy.loginfo("Hand location moved")
    
    def pick(self, grasp, object_name, support_name):

        self.pick_place.pickup(object_name, [grasp, ], support_name = support_name)

    def place(self, parameters={}):

        # "pick_pose" Pose with co-ordinates(point) and orientation(quaternion) for end-effector
        try:
            pick_pose = parameters["place_pose"]
            if type(pick_pose).__name__ != 'Pose':
                raise PoseError(pick_pose)
        except (ValueError, KeyError, PoseError) as e:
            # Pose contains: Point position (x,y,z), Quaternion orientation (x,y,z,w)
            # set to position on Fetch's left side above head
            p = Point(0.53175, 0.71032, 1.26782)
            q = tf.transformations.quaternion_from_euler(np.pi, 0, 0)

            q = Quaternion(q[0],q[1],q[2],q[3])

            pick_pose = Pose(p,q)

        self.__place_object(pick_pose, 0)
    
    # Primitive: pick_and_place - fetch moves arm to a pick pose (close gripper)
    # then moves arm to a place pose (open gripper)
    # Default: pick from pose on Fetch's left side above head
    # place low to ground on left
    def pick_and_place(self, parameters={}):

        # "pick_pose" Pose with co-ordinates(point) and orientation(quaternion) for end-effector
        try:
            pick_pose = parameters["pick_pose"]
            if type(pick_pose).__name__ != 'Pose':
                raise PoseError(pick_pose)
        except (ValueError, KeyError, PoseError) as e:
            p = Point(0.53175, 0.71032, 1.26782)
            q = tf.transformations.quaternion_from_euler(np.pi/2, 0, 0)
            q = Quaternion(q[0],q[1],q[2],q[3])
            pick_pose = Pose(p,q)

        # "place_pose" co-ordinates for end-effector
        try:
            place_pose = parameters["place_pose"]
            if type(place_pose).__name__ != 'Pose':
                raise PoseError(place_pose)
        except (ValueError, KeyError, PoseError) as e:
            # Pose contains: Point position (x,y,z), Quaternion orientation (x,y,z,w)
            # set to position low to ground on left
            p = Point(0.53175, 0.71032, 1.26782)
            q = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
            q = Quaternion(q[0],q[1],q[2],q[3])
            # q = quaternion_from_euler(np.pi, 0, 0)
            #q = Quaternion(0.90357, 0.11173, -0.39944, -0.10731)
            place_pose = Pose(p,q)
            #place_pose = Pose((0.53175, 0.71032, 1.26782,np.pi, 0, 0))

        # "max_gripper_effort" for close/open force of gripper
        try:
            max_gripper_effort = float(parameters["max_gripper_effort"])
        except (ValueError, KeyError) as e:
            max_gripper_effort = FetchGripper.MAX_EFFORT
        if max_gripper_effort < 0.0:
            max_gripper_effort = 0.0
        if max_gripper_effort > FetchGripper.MAX_EFFORT:
            max_gripper_effort = FetchGripper.MAX_EFFORT

        self.__pick_object(pick_pose, max_gripper_effort)
        self.__place_object(place_pose, max_gripper_effort)
        self.tuck_arm()


    def move_while_holding(self, parameters={}):

        # "place_pose" co-ordinates for end-effector
        try:
            place_pose = parameters["move_pose"]
            if type(place_pose).__name__ != 'Pose':
                raise PoseError(place_pose)
        except (ValueError, KeyError, PoseError) as e:
            # Pose contains: Point position (x,y,z), Quaternion orientation (x,y,z,w)
            # set to position low to ground on left

            p = Point(0.53175, 0.65, 1.26782)
            q = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
            q = Quaternion(q[0],q[1],q[2],q[3])

            place_pose = Pose(p,q)
            rospy.sleep(5)

            # "max_gripper_effort" for close/open force of gripper
            

        self.__pick_object(place_pose, 0)
        rospy.sleep(1)
    
    def tilt_while_holding(self, parameters={}):
        self.gripper.close_gripper(30)
        # "place_pose" co-ordinates for end-effector
        try:
            place_pose = parameters["tilt_pose"]
            if type(place_pose).__name__ != 'Pose':
                raise PoseError(place_pose)
        except (ValueError, KeyError, PoseError) as e:
            # Pose contains: Point position (x,y,z), Quaternion orientation (x,y,z,w)
            # set to position low to ground on left

            p = Point(0.53175, 0.65, 1.26782)
            q = tf.transformations.quaternion_from_euler(np.pi/2, 0, 0)
            q = Quaternion(q[0],q[1],q[2],q[3])
            place_pose = Pose(p,q)

        self.__pick_object(place_pose, 0)
        rospy.sleep(1)
    


# contains control for Fetch's gripper
# adapted from Enrique's examples
class FetchGripper(object):

    OPEN_POSITION = 0.10
    CLOSED_POSITION = 0.027
    MAX_EFFORT = 0

    def __init__(self):
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action',
                                    GripperCommandAction)
        rospy.loginfo('Waiting for gripper_controller...')
        self.gripper_client.wait_for_server()
        rospy.loginfo('Found gripper_controller...')

    # set the position of the gripper
    def set_position(self, position, max_effort=None):
        goal = GripperCommandGoal()
        if max_effort == None:
            max_effort = self.MAX_EFFORT
        goal.command.max_effort = max_effort
        goal.command.position = position
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))
        return "success"

    # to open the gripper
    def open_gripper(self, max_effort=None, value=None, parameters=None):
        self.set_position(position = self.OPEN_POSITION, max_effort=max_effort)
        return "success"

    # to close the gripper
    def close_gripper(self, grip_pos, max_effort=None, value=None, parameters=None):
        self.set_position(position = grip_pos, max_effort=max_effort)
        return "success"

# Allows processing of incorrectly entered values
class PoseError(Exception):
    def __init__(self, pose_params):
        self.pose_params = pose_params