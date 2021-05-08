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
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

# move_base imports
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# move_head imports
from control_msgs.msg import PointHeadAction, PointHeadGoal
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib # also used for move_base to pose

# sound imports
from sound_play.libsoundplay import SoundClient
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

# local modules for Fetch primitives
from move_arm.fetch_move_arm import *
from move_base.fetch_move_base import *
from move_head.fetch_move_head import *
from perception.fetch_perception import *
from sound.fetch_sound import *

# Contains an instance of each class needed for primitives
class Robot(object):

    def __init__(self):
        self.arm = FetchArm()
        self.base = FetchBase()
        self.head = FetchHead()
        self.perception = FetchPerceptionVisual()
        self.sound = FetchSound()

    def say(self, value, parameters):
        print(parameters)
        short_paramaters = {'text': value}
        self.sound.speak(short_paramaters)

    # Helper method to call correct primitive (on correct class) from passed in arguments
    # arguments: primitive = string name of primitive
    #           parameters = dictionary of control parameters
    def do_primitive(self, primitive, parameters={}):
        print(primitive)
        print(parameters)
        # Meta programming of available methods for each class of primitives
        # Warning: avoid identical primitive names in different primitive classes e.g. arm vs base
        # An earlier listed class below will overload any identical name in later primitive classes
        primitive_classes = [self, self.arm, self.base, self.head,
            self.perception, self.sound]

        for primitive_class in primitive_classes:
            if primitive in dir(primitive_class):
                # method exists for this class, so call it with the other parameters
                getattr(primitive_class, primitive)(parameters)
                # reset arm position if an arm primitive was called
                if primitive_class == self.arm:
                    self.arm.tuck_arm()
                #return ("successfully found primitive: " + primitive)
                # simplified return values to integrate with RIZE
                return "success"

        # no primitive found in any primitive class
        rospy.logerr("No primitive of name '" + primitive + "' exists")
        return "failure"

    # wake_up primitive - Fetch moves around on the spot as if waking up ready for instructions
    def wake_up(self, parameters={}):
        # head move down
        self.head.tilt_eyes({"direction": 'down', "angle_deg": 90})

        # base move left then right then centre
        self.base.turn_base({"direction": 'left', "angle_deg": 45})
        self.base.turn_base({"direction": 'right', "angle_deg": 45})
        self.base.turn_base({"direction": 'right', "angle_deg": 45})
        self.base.turn_base({"direction": 'left', "angle_deg": 45})

        # head move up
        self.head.tilt_eyes({"direction": 'up', "angle_deg": 45})

        # torso move up then down
        self.arm.move_torso()

        self.head.reset_head()
        rospy.loginfo("Fetch is awake!")

    # rest primitive
    def rest(self, parameters={}):
        self.base.stop_base()
        self.head.reset_head()
        self.arm.tuck_arm()
        rospy.loginfo("Fetch is resting!")

    # wait_idle primitive - stops all actions and stays in current pose for "seconds"
    def wait_idle(self, parameters={}):
        # pauses for the time "seconds" default 3 sec
        try:
            seconds = int(parameters["seconds"])
        except (ValueError, KeyError) as e:
            seconds = 3

        self.base.stop_base()
        self.arm.stop_arm()
        self.head.reset_head()
        self.sound.stop_sound()
        rospy.loginfo("Fetch is idling for %s seconds", seconds)
        rospy.sleep(seconds)

    def walk(self, value, parameters={}):
        short_paramaters = {'metres': parameters['meters']}

        if value in ('forwards', 'forward'):
            self.do_primitive('move_forward', short_paramaters)
        elif value in ('backward', 'backwards'):
            self.do_primitive('move_backward', short_paramaters)
        else:
            print('Unknown parameters')

        return "success"

    def turn(self, value, parameters={}):
        short_paramaters = {'angle_deg': parameters['degrees']}

        if value == 'right':
            short_paramaters['direction'] = 'right'
        else:
            short_paramaters['direction'] = 'left'

        self.do_primitive('turn_base', short_paramaters)

    def do_animation(self, value, parameters={}):
        value_formatted = value.replace("-", "_")
        short_paramaters = {"repeat": parameters['time'], 'reverse': parameters['reverse']}
        self.do_primitive(value_formatted, short_paramaters)