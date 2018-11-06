  
from lab2 import Robot
import rospy


if __name__ == '__main__':

	rospy.init_node('lab2')
	R = Robot()
	print "lab2 launch,  "  

	rospy.sleep(0.5)
	R.drive_straight(2,0)
	rospy.sleep(0.2)
	R.rotate( 3.14159 )

	while not R.shutFlag:
		print "\nx y head " , R.current.x , R.current.y,R.current.heading , " time " , rospy.Time.now()
		print " desire X Y head " ,R.desire.x , R.desire.y , R.desire.heading , " seq " , R.desire.seq
		print " heading : "
		print R.anglesTest
		
		rospy.sleep(2)