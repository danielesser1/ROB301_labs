#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np

def get_scan():
        scan = rospy.wait_for_message('scan', LaserScan)
       
        #samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        #samples_view = 1            # 1 <= samples_view <= samples

        #print("value at 0 degrees:{}".format(scan.ranges[0]))
        #print("value at 90 degrees:{}".format(scan.ranges[90]))
        #print("value at 270 degrees:{}".format(scan.ranges[270]))
	#   0 degree
	#       |
	# 90 ------- 270
        scan_filter = []
        for val in scan.ranges:
            if val == 0:
                scan_filter.append(30)
            else:
                scan_filter.append(val)

	closest = []
	for i in range(10):
		ind = np.argmin(scan_filter)
		closest.append(ind)
		scan_filter[ind] = 100
	#print(np.mean(closest))
        
	#derivative = [0]
        #for i in range(1, len(scan_filter)):
        #    derivative.append(abs(scan_filter[i]-scan_filter[i-1]))
	
        #edge_1 = np.argmax(derivative)
        #derivative[edge_1] = 0
        #edge_2 = np.argmax(derivative)
	#print(edge_1, edge_2)
        #print((edge_1 + edge_2)/2)
    	return np.mean(closest)

def main():
    rospy.init_node('get_scan_angle')
    scan_pub = rospy.Publisher('scan_angle', String, queue_size=1)
    try:
    	while(1):
        	ind = get_scan()
        	scan_pub.publish(str(ind))
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
