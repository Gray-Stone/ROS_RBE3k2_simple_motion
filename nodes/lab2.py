#!/usr/bin/env python

import copy
import math 
import numpy as np
import rospy
import tf 

from geometry_msgs.msg import (PoseStamped, Twist, Pose )
# somehow these line import fine but can't do rospack find Pose 
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

   


class Robot:
    # @dataClass
    class poseStruct:
        x =0
        y =0
        heading =0
        seq =0 # this is to keep count of which desire position is used

    current = poseStruct()
    desire = poseStruct()
    shutFlag = False

    anglesTest =None

    class valCmdC :
        pub = rospy.Publisher('cmd_vel',Twist,queue_size=2)
        msg = Twist()
        def push(self):
            self.pub.publish(self.msg)

    valCmd = valCmdC()
    
    def __init__(self):
        """"
        Set up the node here
        """
        # subscrib to odemtry and update stored data
        rospy.Subscriber("odom",Odometry,self.odom_callback)
        # subscrib to simple goal and move the robot to it when it's called
        rospy.Subscriber("move_base_simple/goal",PoseStamped,self.nav_to_pose)

        rospy.on_shutdown(self.shutdown)

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        self.desire.seq= goal.header.seq
        self.desire.x = goal.pose.position.x
        self.desire.y = goal.pose.position.y
        quat = goal.pose.orientation
        self.desire.heading = self.quat2theta(quat)

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive shraight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """
        # setting speed will be setting to cmd_vel 
        # type is geometry_msgs/Twist

        # TODO tell it to stop at some point 

        self.valCmd.msg.linear.x = speed 
        self.valCmd.push()
        rospy.sleep(2.2)
        self.valCmd.msg.linear.x = 0
        self.valCmd.push()

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        pause = angle
        self.valCmd.msg.angular.z = 1
        self.valCmd.push()
        rospy.sleep(pause)
        self.valCmd.msg.angular.z = 0
        self.valCmd.push()

    def odom_callback(self, msg):

        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.current.x = msg.pose.pose.position.x
        quat = msg.pose.pose.orientation # this return a Quaternion 
        self.current.heading = self.quat2theta(quat)
        self.current.y = msg.pose.pose.position.y

    def quat2theta (self, quat):
        heading = euler_from_quaternion([quat.x , quat.y ,quat.z ,quat.w])[2]
        if heading <0:
            heading += math.pi*2 
        if heading > math.pi*2:
            heading -= math.pi*2
        return heading

    def shutdown(self):
        # nothing yet 
        self.desire.seq=999
        print "\n\nshutting down (should be)\n\n"
        self.shutFlag=True

# if __name__=="__main__":
    

    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # while not rospy.is_shutdown():    


if __name__ == '__main__':
    rospy.init_node('lab2')
    R = Robot()
    x=10
    print "lab2.py main " , x 

    rospy.sleep(1)
    R.drive_straight(2,0)
    rospy.sleep(0.5)
    R.rotate( 3.14159 )

    while not R.shutFlag:
        print "\nx y head " , R.current.x , R.current.y,R.current.heading , " time " , rospy.Time.now()
        print " desire X Y head " ,R.desire.x , R.desire.y , R.desire.heading , " seq " , R.desire.seq
        print " heading : "
        print R.anglesTest
        
        rospy.sleep(2)


    # first listen to /move_base_simple/goal of type geometry_msgs/PoseStamped
    # rostopic info /move_base_simple/goal 
    # Type: geometry_msgs/PoseStamped

    # Publishers: 
    #  * /rviz (http://localhost:34215/)
