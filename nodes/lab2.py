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
        seq =-1 # this is to keep track of updating
        def __init__(self,x,y,heading):
            self.x=x
            self.y=y
            self.heading = heading

    current = PoseStruct(0,0,0)     # this is what Odem call back does. 
    desire = PoseStruct(0,0,0)      # controller will run base on these 
    reachedFlag = PoseStruct(True,True,True)
    tolerance = PoseStruct(0.07,0.07, 0.5/180*math.pi )

    class ValCmdStruct :
        pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        msg = Twist()
        def push(self):
            self.pub.publish(self.msg)
        def setZ(self , speed):
            self.msg.angular.z = speed 
        def setX(self, speed):
            self.msg.linear.x=speed 
    valCmd = ValCmdStruct()

    class CheckType:
        Angle =1 
        Pos  =2 
    
    def __init__(self):
        """"
        Set up the node here
        """
        rospy.loginfo(   "init the Robot command system")
        # subscrib to odemtry and update stored data
        rospy.Subscriber("odom",Odometry,self.odom_callback)
        # subscrib to simple goal and move the robot to it when it's called
        rospy.Subscriber("move_base_simple/goal",PoseStamped,self.navPoint_callback)
        # make sure it does shutdown when needed
        rospy.on_shutdown(self.shutdown)

        self.valCmd.msg.linear.x = 0
        self.valCmd.msg.angular.z=0
        self.valCmd.push()

    def nav_to_pose(self):
    # control loop until robot goes to final pose
        # load final into desire 

        xDiff = self.desire.x - self.current.x
        yDiff = self.desire.y - self.current.y
        toGoalHeading = math.atan2(yDiff,xDiff)
        # rospy.logdebug("calculated direction toward goal is %.2f", toGoalHeading/math.pi*180)

        # if the robot is too much off in heading
        while abs(toGoalHeading)>math.pi/2:
            xDiff = self.desire.x - self.current.x
            yDiff = self.desire.y - self.current.y
            toGoalHeading = math.atan2(yDiff,xDiff)
            self.rotate(toGoalHeading)
            self.valCmd.push()
        self.stopMoving()
        rospy.sleep(1)

        # move robot toward goal, and turning at the same time 
        while self.shutFlag == False:
            if self.drive_straight() :
                break 
            xDiff = self.desire.x - self.current.x
            yDiff = self.desire.y - self.current.y
            toGoalHeading = math.atan2(yDiff,xDiff)
            self.rotate(toGoalHeading)
            self.valCmd.push()
        self.stopMoving()

        while self.shutFlag


        
            

    def navPoint_callback(self,goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        if self.desire.seq == goal.header.seq : # this mean it's the same old package, nothing need to be done
            return 
        self.desire.seq = goal.header.seq
        self.desire.x = goal.pose.position.x
        self.desire.y = goal.pose.position.y
        quat = goal.pose.orientation
        self.desire.heading = self.quat2theta(quat)


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

    def drive_straightTest(self, distance):
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
        rospy.loginfo("start driving test ")

        # move in the same direction as robot
        self.desire.x = self.current.x + distance* math.cos(self.current.heading)

        self.reachCheck(self.CheckType.Pos)
        while not self.reachedFlag.x :
            self.drive_straight()
            self.valCmd.push()
            if self.shutFlag :
                break

        rospy.loginfo("end driving test, location %.2f %.2f %.2f :" , self.current.x , self.current.y,self.current.heading)
   
    def drive_straight(self):
        """ actually control the robot forwared 
        """ 
        self.reachCheck(self.CheckType.Pos)
        if self.reachedFlag.x  and self.reachedFlag.y:        
            self.valCmd.msg.linear.x = 0
            self.valCmd.msg.angular.z = 0
            return True
        
        pGain = 0.5
        xDiff = self.desire.x - self.current.x
        yDiff = self.desire.y - self.current.y
        dis = math.sqrt( xDiff*xDiff + yDiff*yDiff)        
        moveSpeed = pGain*dis
        if moveSpeed>0.3 :
            moveSpeed = 0.3
        if moveSpeed<0.02 :
            moveSpeed = 0.02

        rospy.logdebug("disDiff is %.2f , moveSpeed :%.2f" , dis,moveSpeed)
        # prefvent sending too much package
        speedDiff = moveSpeed - self.valCmd.msg.linear.x 
        # if abs(speedDiff) > 0.02 :
        #     self.valCmd.msg.linear.x = moveSpeed
        #     self.valCmd.push()
        # return False
        self.valCmd.msg.linear.x = moveSpeed
        return False 

    def rotateTest(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        self.desire.heading=angle       # store the current angle
        self.reachCheck(self.CheckType.Angle)  
        while not self.reachedFlag.heading  : # keep controlling itself until reached. 
            self.rotate(angle)
            self.valCmd.push()
            if self.shutFlag :
                break
    
    def rotate(self,angle):    
    # return True when it's done turning, return false when it's not done turning. 
       
        if  abs(angle - self.current.heading) < self.tolerance.heading : 
            self.valCmd.msg.angular.z = 0
            return True

        # this section assume negative angle will turn right and negative twist will also turn right
        # turning speed should be from 2.0 to 0.1
        # assume at 0.5rad(28deg) reach max speed 
        pGain = 2
        # rospy.logdebug("angle C : %.2f D: %.2f ",self.current.heading,self.desire.heading)
        angleDiff= angle - self.current.heading   # calculate the amount of turnning needed
        angleDiff = self.angleFix(angleDiff)                                # fit them within -180 to 180 
       

        turnSpeed = angleDiff * pGain    
        if abs(turnSpeed)>0.7 :
            turnSpeed = math.copysign(0.7,turnSpeed)
        if abs(turnSpeed)<0.1 :
            turnSpeed = math.copysign(0.1,turnSpeed)

        rospy.logdebug("turnSpeed: %.2f" , turnSpeed )

        if abs( self.valCmd.msg.angular.z - turnSpeed) >0.05 :
            self.valCmd.msg.angular.z = turnSpeed

#####################           Helper 
###################################################
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
        # which should be one of the thing in CheckType 

        if  abs(self.desire.x - self.current.x) < self.tolerance.x : 
            self.reachedFlag.x = True  
        else: 
            self.reachedFlag.x = False 

        if  abs(self.desire.y - self.current.y) < self.tolerance.y : 
            self.reachedFlag.y = True  
        else: 
            self.reachedFlag.y = False 

        if  abs(self.desire.heading - self.current.heading) < self.tolerance.heading : 
            self.reachedFlag.heading = True  
        else: 
            self.reachedFlag.heading = False 

    def stopMoving(self):
        self.valCmd.msg.angular.z=0
        self.valCmd.msg.linear.x =0
        self.valCmd.push()

    def shutdown(self):
        # nothing yet 
        self.desire.seq=999
        print "\n\nshutting down (should be)\n\n"
        self.shutFlag=True
    def getTime(self):
        pass
        # rospy.Time.now
 


if __name__ == '__main__' : 

    rospy.init_node('lab2Goal',log_level=rospy.DEBUG)
    # rospy.init_node('lab2Goal',log_level=rospy.INFO)

    R = Robot()
    rospy.loginfo("init the robot comminder")
    rospy.sleep(2)
    rospy.loginfo("stopping robot ")
    R.stopMoving()
    rospy.sleep(2)
    rospy.loginfo("directly for moving to goal") 
    
    while True :
        R.nav_to_pose()
        rospy.sleep(2)


