#!/usr/bin/env python

# standard ROS imports
from __future__ import print_function
import sys
import copy
import time
import math
import rospy
from std_msgs.msg import ColorRGBA, Float32
from six.moves import input
import visualization_msgs.msg
import actionlib
import numpy as np
import tf2_ros
import tf
import tf_conversions
from tuck_arm import TuckThread

# move_arm imports
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, Grasp, PlaceLocation, RobotState, Constraints, OrientationConstraint
from moveit_python import *
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Joy
import sensor_msgs.msg
from manipulation.srv import *

class FetchArm(object):

    MAX_TORSO = 0.4
    MIN_TORSO = 0
    tuck_arm_pos = [0.4, np.deg2rad(90), 1.40, 0.0, 1.72, 0.0, np.deg2rad(90), np.deg2rad(90)]
    MAX_VELOCITY_SCALING_FACTOR = 0.2

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
        self.init_robot_self_collision_avoidanace()

        # Move commander and Move group initilization
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")
        self.move_commander = moveit_commander.MoveGroupCommander("arm_with_torso")
        self.robot = moveit_commander.RobotCommander()
        self.pick_place = PickPlaceInterface("arm", "gripper")
        self.move_commander.set_end_effector_link("gripper_link")

        # Init max speed
        self.move_commander.set_max_velocity_scaling_factor(self.MAX_VELOCITY_SCALING_FACTOR)

        # Create a publisher to visualize the position constraints in Rviz
        self.marker_publisher = rospy.Publisher(
            "/visualization_marker", visualization_msgs.msg.Marker, queue_size=20,
        )
        rospy.sleep(0.5)  # publisher needs some time to contect Rviz
        self.remove_all_markers()
        self.marker_id_counter = 0  # give each marker a unique idea

        # create a gripper object
        self.gripper = FetchGripper()

        # check for controller input
        self.pressed = False
        self.pressed_last = None
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

        rospy.loginfo("Fetch move arm initialised")

    def remove_all_markers(self):
        """ Utility function to remove all Markers that we potentially published in a previous run of this script. """
        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        # marker.id = 0
        # marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        self.marker_publisher.publish(marker)

    def create_start_state(self):
        """ Create a RobotState message from a named joint target for this robot. """
        ready = self.move_commander.get_named_target_values("ready")

        # Now create a robot state from these joint positions
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = self.move_commander.get_pose_reference_frame()
        joint_state.name = [key for key in ready.keys()]
        joint_state.position = [val for val in ready.values()]

        #state = moveit_msgs.msg.RobotState()
        #state.joint_state = joint_state
        state = self.move_commander.get_current_state()
        rospy.loginfo(state)
        return state

    def solve_path_plan(self, pose_goal):
        """ Solve path plan and execure with constraints applied"""
        # Clear 
        self.move_commander.clear_pose_targets()

        self.move_commander.set_start_state_to_current_state()
        self.move_commander.set_pose_target(pose_goal,"gripper_link")

        path = self.move_commander.plan()

        path_retimed = self.move_commander.retime_trajectory(self.move_commander.get_current_state(),path,self.MAX_VELOCITY_SCALING_FACTOR)            
        self.move_commander.execute(path_retimed)  

        # TODO: collision checking so controller doesn't shutdown
        
    def init_upright_constraint(self, tolerance_deg, constrained_axis):
        """ Initialise upright constraint with tolerance in degrees for all axes. """
        # Axis to constrain
        if constrained_axis == "x":
            constrained_orientation = euler_to_quat([0,0,0])
        elif constrained_axis == "z":
            constrained_orientation = euler_to_quat([0,np.deg2rad(90),0])
        else:
            return "Error: Invalid axis"

        self.move_commander.clear_pose_targets()
        self.move_commander.clear_path_constraints()
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        constraints = moveit_msgs.msg.Constraints()

        tolerance = np.deg2rad(tolerance_deg)
        start_pose = self.move_commander.get_current_pose("gripper_link")

        constraints.name = "upright"
        self.move_commander.set_pose_reference_frame("base_link")
        orientation_constraint.header = start_pose.header
        orientation_constraint.link_name = "gripper_link"
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.orientation = constrained_orientation
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orientation_constraint)
        self.move_commander.set_path_constraints(constraints)
        rospy.loginfo("Upright constraint applied with %s deg tolerance",tolerance_deg)
        #rospy.loginfo(self.arm.move_commander.get_path_constraints())

    def init_robot_self_collision_avoidanace(self):
        self.planning_scene.removeCollisionObject("ground_keepout")
        self.planning_scene.removeCollisionObject("base_keepout_1")
        self.planning_scene.removeCollisionObject("base_keepout_2")
        self.planning_scene.addBox("ground_keepout", 1.5, 1.5, 0.02, 0.0, 0.0, -0.012)
        self.planning_scene.addBox("base_keepout_1", 0.25, 0.45, 0.001, 0.09, 0.0, 0.368)
        self.planning_scene.addCylinder("base_keepout_2",0.001, 0.15, 0.12, 0, 0.368)
        self.planning_scene.addBox("base_side_keepout_1",0.4,0.001,0.35,0.02,0.27,0.2)
        self.planning_scene.addBox("base_side_keepout_2",0.4,0.001,0.35,0.02,-0.27,0.2)
        self.planning_scene.addBox("front_keepout", 0.001, 0.45, 0.3, 0.29, 0.0, 0.2)

    def cartesian_path(self, axis_position, axis):
        """ Function for movement along single axis. axis: {x, y, z}. direction: {true: positive direction, false: negative direction} """
        self.move_commander.clear_pose_targets()
        self.move_commander.clear_path_constraints()

        waypoints = []
        segments = 5

        wpose = self.move_commander.get_current_pose("gripper_link").pose 

        # Take the differnce between current and desired position and segment for waypoints
        if axis =="x":
            dif = axis_position-wpose.position.x
        elif axis =="y":
            dif = axis_position-wpose.position.y
        elif axis =="z":
            dif = axis_position-wpose.position.z
        else:
            return "Error: Invalid axis"
        
        for i in range(0,segments):
            if axis =="x":
                    wpose.position.x += dif/segments  
            elif axis =="y":
                    wpose.position.y += dif/segments  
            elif axis =="z":
                    wpose.position.z += dif/segments  
            waypoints.append(copy.deepcopy(wpose))

        #rospy.loginfo(waypoints)

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:

        (plan, fraction) = self.move_commander.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        rospy.loginfo("Waypoint fraction: %s",fraction)
        
        # Set max path speed
        plan_retimed = self.move_commander.retime_trajectory(self.move_commander.get_current_state(),plan,self.MAX_VELOCITY_SCALING_FACTOR)
        self.move_commander.execute(plan,wait=True)

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

    # Primitive: tuck_arm - reset arm to idle pose
    def tuck_arm(self, lower_torso=False, parameters={}):
        
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
        
        if lower_torso:
            # Move torso down
            self.planning_scene.clear()
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

# Euler to quaterion rotation-object conversion
def euler_to_quat(euler_angles):
    hold_quat_array =tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1] , euler_angles[2])
    quat_object= Quaternion(hold_quat_array [0], hold_quat_array [1], hold_quat_array [2], hold_quat_array [3])
    return quat_object