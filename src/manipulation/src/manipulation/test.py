#!/usr/bin/env python
import rospy
from manipulation.manipulation_node import RobotManipulation
import geometry_msgs.msg 

def main():
    rospy.init_node('test')
    rospy.loginfo('Start test')
    robotManipulation = RobotManipulation()
    #robotManipulation.arm.tuck_arm()
    robotManipulation.planningScene()

    #eef_pos = robotManipulation.pickUpCup()
    #eef_pos = robotManipulation.moveToMachine(eef_pos)
    #robotManipulation.openHatch(eef_pos)
    [grip_loc, grip_or] = robotManipulation.currGripperLocation()


if __name__ == '__main__':
    main()
