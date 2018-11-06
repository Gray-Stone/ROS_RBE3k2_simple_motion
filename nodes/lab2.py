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
    shutFlag = False
    
    # @dataClass
    class PoseStruct:
        x =None
        y =None
        heading =None
        seq =0 # this is to keep track of updating
        def __init__(self,x,y,heading):
            self.x=x
            self.y=y
            self.heading = heading

    current = PoseStruct(0,0,0)
    desire = PoseStruct(0,0,0)
    reachedFlag = PoseStruct(True,True,True)
    tolerance = PoseStruct(0.1,0.1,0.5)

    class ValCmdStruct :
        pub = rospy.Publisher('cmd_vel',Twist,queue_size=2)
        msg = Twist()
        def push(self):
            self.pub.publish(self.msg)
    valCmd = ValCmdStruct()
    
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
        if self.desire.seq == goal.header.seq : # this mean it's the same old package, nothing need to be done
            return 
        self.desire.x = goal.pose.position.x
        self.desire.y = goal.pose.position.y
        quat = goal.pose.orientation
        self.desire.heading = self.quat2theta(quat)
        self.reachCheck()

        

    def odom_callback(self, msg):
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.current.x = msg.pose.pose.position.x
        self.current.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation # this return a Quaternion 
        self.current.heading = self.quat2theta(quat)
        self.current.seq=msg.header.seq     # make sure the numbers are recorded

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
        self.desire.heading=angle       # store the current angle
        while not self.reachedFlag.heading  : # keep controlling itself until reached. 
            self.rotateControl()
            self.reachCheck()            

    def rotateControl(self):    
        # this section assume negative angle will turn right and negative twist will also turn right
        # turning speed should be from 2.0 to 0.1
        # assume at 0.5rad(28deg) reach max speed 
        pGain = 4

        angleDiff= self.current.heading - self.desire.heading   # calculate the amount of turnning needed
        self.angleFix(angleDiff)                                # fit them within -180 to 180 
       
        turnSpeed = angleDiff * pGain    
        if abs(turnSpeed)>2 :
            turnSpeed = math.copysign(2,turnSpeed)
        if abs(turnSpeed)<0.2 :
            turnSpeed = math.copysign(0.2,turnSpeed)

        self.valCmd.msg.angular.z=turnSpeed
        self.valCmd.push()

    def quat2theta (self, quat):
        # this function strip out the quaternion from pose into the angle in Z 
        heading = euler_from_quaternion([quat.x , quat.y ,quat.z ,quat.w])[2]
        return self.angleFix(heading)
       
    def angleFix (self,angle):
        # change the input angle to within the -pi to pi range. 
        # type: msg: number in rad
        if angle <-math.pi:
            angle += math.pi*2 
        if angle > math.pi:
            angle -= math.pi*2
        return angle

    def reachCheck(self):
        # check to see if robot has reached the goal
        if  (self.desire.x - self.current.x) < self.tolerance.x : 
            self.reachedFlag.x = True  
        else: 
             self.reachedFlag.x = False 
        if  (self.desire.y - self.current.y) < self.tolerance.y : 
            self.reachedFlag.y = True  
        else: 
             self.reachedFlag.y = False 
        if  (self.desire.heading - self.current.heading) < self.tolerance.heading : 
            self.reachedFlag.heading = True  
        else: 
             self.reachedFlag.heading = False 

            

    def shutdown(self):
        # nothing yet 
        self.desire.seq=999
        print "\n\nshutting down (should be)\n\n"
        self.shutFlag=True

# if __name__=="__main__":
    

    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # while not rospy.is_shutdown():    


if __name__ == '__main__':

    print "lab2.py main and nothing to be done " 



    # first listen to /move_base_simple/goal of type geometry_msgs/PoseStamped
    # rostopic info /move_base_simple/goal 
    # Type: geometry_msgs/PoseStamped

    # Publishers: 
    #  * /rviz (http://localhost:34215/)
