#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def publisher_node():
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()
    start = time.time()
    while time.time() - start <= 5:
        twist.linear.x = 0.1
        cmd_pub.publish(twist)
    twist.linear.x = 0
    start = time.time()
    cmd_pub.publish(twist)
    while time.time()-start <=2:
        twist.angular.z = 0.1
        cmd_pub.publish(twist)
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_pub.publish(twist) #stop
        


def main():
    
    try:
        rospy.init_node('publisher')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


