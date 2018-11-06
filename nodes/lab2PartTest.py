#!/usr/bin/env python
 
from lab2 import Robot
import math
import rospy

def printStatus(R):
	print "\ncurrent X: %.2f \t Y: %.2f \t head: %.2f" % (R.current.x , R.current.y,R.current.heading)
	print "desire  X: %.2f \t Y: %.2f \t head: %.2f \t seq: %d" % (R.desire.x , R.desire.y , R.desire.heading , R.desire.seq)
	print " reached x:%r y:%r head: %r : " % (R.reachedFlag.x , R.reachedFlag.y , R.reachedFlag.heading)
		

if __name__ == '__main__':

	rospy.init_node('lab2PartTest',log_level=rospy.DEBUG) 
	# rospy.init_node('lab2Run', log_level=rospy.INFO) 

	R = Robot()
	print "lab2 robot launch "  
	rospy.loginfo("waiting to let robot connect")
	rospy.sleep(2)



	R.valCmd.msg.linear.x = 0
	R.valCmd.msg.linear.y = 0
	R.valCmd.msg.angular.z=0
	R.valCmd.push()

	printStatus(R)

	rospy.loginfo("\nRobot is commanded to stop")

	rospy.sleep(2)
	
	printStatus(R)
	distance = 0.25
	rospy.loginfo("start driving straight for %.3f ",distance)
	R.drive_straightTest(distance)
	rospy.loginfo("done driving straight")
	printStatus(R)
	rospy.sleep(2)

	headings = [ math.pi/2 , 0 , math.pi/4 , -math.pi/4 ]

	for heading in headings :
		rospy.loginfo("rotate test toward :%.3f",heading)
		rospy.sleep(0.5)
		R.rotateTest(heading)
		printStatus(R)
		rospy.sleep(0.5)

