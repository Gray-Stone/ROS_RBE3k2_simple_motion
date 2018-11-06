#!/usr/bin/env python
 
from lab2 import Robot
import math
import rospy

def printStatus(R):
	print "\ncurrent X: %.2f \t Y: %.2f \t head: %.2f" % (R.current.x , R.current.y,R.current.heading)
	print "desire  X: %.2f \t Y: %.2f \t head: %.2f \t seq: %d" % (R.desire.x , R.desire.y , R.desire.heading , R.desire.seq)
	print " reached x:%r y:%r head: %r : " % (R.reachedFlag.x , R.reachedFlag.y , R.reachedFlag.heading)
		

if __name__ == '__main__':

	# rospy.init_node('lab2Run',log_level=rospy.DEBUG) 
	rospy.init_node('lab2Run') 
	print "lab2 launch "  

	R = Robot()

	R.drive_straightTest(5)
	rospy.sleep(0.5)



	R.rotateTest( math.pi )
	rospy.sleep(1)
	R.rotateTest(math.pi/2)
	rospy.sleep(1)
	R.rotateTest(math.pi/3)
	rospy.sleep(1)
	R.rotateTest(-math.pi/4)

	while not R.shutFlag:
		printStatus(R)
		rospy.sleep(2)
