#!/usr/bin/env python
import rospy
from manipulation.manipulation_node import RobotManipulation
import geometry_msgs.msg 
from fetch_move_arm import FetchArm

def main():
    rospy.init_node('test')
    rospy.loginfo('Start test')
    robotManipulation = RobotManipulation()

    eef_pos = robotManipulation.pickUpCup()
    
    #eef_pos = robotManipulation.moveToMachine(eef_pos)
    #robotManipulation.openHatch(eef_pos)



if __name__ == '__main__':
    main()
