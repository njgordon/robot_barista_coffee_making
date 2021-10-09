#!/usr/bin/env python

# standard ROS imports
import sys
import time
import math

#from copy import copy #move_head by joint angle
import rospy
from std_msgs.msg import ColorRGBA, Float32

# move_head imports
from control_msgs.msg import PointHeadAction, PointHeadGoal
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib # also used for move_base to pose

# contains all Fetch primitives related to head movement
class FetchHead(object):
    MAX_HEAD_PAN_DEG = 89
    MAX_HEAD_TILT_UP = 45
    MAX_HEAD_TILT_DOWN = 80

    # init adapted from Fetch manual
    def __init__(self):
        # setup for head point-to primitives
        self.point_head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        self.point_head_client.wait_for_server()

        # setup for head joint-movement primitives - NOT IN USE as buggy
        #self.follow_joint_head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        #self.follow_joint_head_client.wait_for_server()
        #self.joint_names = ["head_pan_joint", "head_tilt_joint"]

    # Primitive: move_head_to - move head/eyes to co-ordinate position
    # default zero pose
    def move_head_to(self, parameters={}):
        # "position" co-ordinates
        try:
            head_position = parameters["position"]
        except (ValueError, KeyError) as e:
            head_position = [1.0, 0.0, 1.0] #x,y,z - relative to base (frame_id)

        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = "base_link"
        goal.target.point.x = head_position[0]
        goal.target.point.y = head_position[1]
        goal.target.point.z = head_position[2]
        self.point_head_client.send_goal(goal)
        self.point_head_client.wait_for_result()
        rospy.sleep(0.5)
        rospy.loginfo("Head moved to %s", head_position)

    # Primitive: reset_head - move head/eyes to default zero position
    def reset_head(self, parameters={}):
        # use move_head_to with zero position
        self.move_head_to({"position": [1.0, 0.0, 1.0]})
        rospy.loginfo("Head position reset")

    # Primitive: turn_head - turn 'direction' right(default) or left
    # by 'angle_deg'(default 45) degrees
    # head_pan_joint limits: +/- 90deg
    def turn_head(self, parameters={}):
        # "direction" string to turn
        try:
            direction = str(parameters["direction"])
        except (ValueError, KeyError) as e:
            direction = "right"
        if direction not in ["right","left"]:
            direction = "right"

        # "angle_deg" to turn by
        try:
            angle_deg = int(parameters["angle_deg"])
        except (ValueError, KeyError) as e:
            angle_deg = 45
        if angle_deg > self.MAX_HEAD_PAN_DEG:
            angle_deg = self.MAX_HEAD_PAN_DEG
        if angle_deg < 0:
            angle_deg = 0

        # Process head turning via joint movements WIP buggy
        #goal = FollowJointTrajectoryGoal()
        #point = JointTrajectoryPoint()
        # joints are given in radians from centre 0
        #positions = [1.5,0.0] #x(left +ve, right -ve),y
        #point.positions = copy(positions)
        # maps the joints in joint_names to angles (radians) in pose
        #goal.trajectory.joint_names = self.joint_names
        #goal.trajectory.points.append(point)
        #goal.trajectory.header.stamp = rospy.Time.now()
        #for i in range(5):
        #    self.follow_joint_head_client.send_goal(goal)
        #rospy.sleep(3)
        #self.follow_joint_head_client.cancel_goal()

        # Use 'head point to' and trig to move head in angles:
        angle_rad = math.radians(angle_deg)
        y = math.tan(angle_rad)
        if direction == "right":
            y = -y
        self.move_head_to({"position": [1.0, y, 1.0]})
        rospy.loginfo("Turned head %s by %s degrees", direction, angle_deg)

    # Primitive: tilt_eyes - tilt 'direction' down(default) or up
    # by 'angle_deg'(default 45) degrees
    # head_tilt_joint limits: 90deg (down), 45deg (up)
    # def move_head_to(self, parameters={}):
        # "position" co-ordinates
    def tilt_eyes(self, parameters={}):
        # "direction" string to turn
        try:
            direction = str(parameters["direction"])
        except (ValueError, KeyError) as e:
            direction = "down"
        if direction not in ["down","up"]:
            direction = "down"

        # "angle_deg" to tilt by
        try:
            angle_deg = int(parameters["angle_deg"])
        except (ValueError, KeyError) as e:
            angle_deg = 45
            
        if angle_deg < 0:
            angle_deg = 0
            
        if direction == "up":
            if angle_deg > self.MAX_HEAD_TILT_UP:
                angle_deg = self.MAX_HEAD_TILT_UP
        else: # tilt down
            if angle_deg > self.MAX_HEAD_TILT_DOWN:
                angle_deg = self.MAX_HEAD_TILT_DOWN
            angle_deg = -angle_deg

        # Use 'head point to' and trig to move head in angles:
        angle_rad = math.radians(angle_deg)
        z = math.tan(angle_rad) + 1
        self.move_head_to({"position": [1.0, 0.0, z]})

        rospy.loginfo("Tilted head(eyes) %s by %s degrees", direction, angle_deg)

    # Primitive: shake head as if to say "NO"
    def shake_head_no(self, parameters={}):
        # repeats primitive for the number specificed by "repeat" param
        try:
            repeat = int(parameters["repeat"])
        except (ValueError, KeyError) as e:
            repeat = 1
        # move head left to right and back to centre
        for x in range(repeat):
            self.move_head_to({"position": [1.0, 5.0, 1.0]})
            self.move_head_to({"position": [1.0, -5.0, 1.0]})
            rospy.loginfo("Shake head #%s successful!", x+1)
        self.reset_head()

    # Primitive: nod head/eyes as if to say "YES"
    def nod_head_yes(self, parameters={}):
        # repeats primitive for the number specificed by "repeat" param
        try:
            repeat = int(parameters["repeat"])
        except (ValueError, KeyError) as e:
            repeat = 1

        # move head down and back to centre
        for x in range(repeat):
            self.move_head_to({"position": [1.0, 0.0, -4.0]})
            self.reset_head()
            rospy.loginfo("Nod head #%s successful!", x+1)