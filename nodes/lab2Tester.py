#!/usr/bin/env python

import math 
import numpy as np


from lab2 import Robot
# R =Robot()
# R.pose[0] = 10;
# print R.pose
# R.heading
def currecthead(heading):
	if heading <0:
		heading += math.pi*2 
	if heading > math.pi*2:
		heading -= math.pi*2
	return heading

a= math.pi
print "something , \t %.2f \t %.2f : " % (a ,a )

# angle = math.pi*0.25
# print "cos"
# print "0.5pi  ", math.cos(angle)
# print "-0.5pi ", math.cos(-angle)

# print "sin"
# print "0.5pi  ", math.sin(angle)
# print "-0.5pi ", math.sin(-angle)

# angle = currecthead(angle)
# print "corrected"
# print "cos"
# print "0.5pi  ", math.cos(angle)
# print "-0.5pi ", math.cos(-angle)

# print "sin"
# print "0.5pi  ", math.sin(angle)
# print "-0.5pi ", math.sin(-angle)

