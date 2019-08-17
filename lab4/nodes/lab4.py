#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KalmanFilter(object):
    def __init__(self, A = None, B = None, D = None, Q = None, R = None, P = None, x0 = None, h=None, d=None):

        if(A is None or D is None):
            raise ValueError("Set proper system dynamics.")

        self.h = np.reshape(np.array([h]),(1,1)) #y
        self.d = np.reshape(np.array([d]),(1,1)) #x

        self.n = A.shape[1]
        self.A = A
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

        self.scan_sub = rospy.Subscriber('scan_angle', String, self.scan_call_back, queue_size=1)
        self.u = 0
        self.phi = 0

        self.measurements = []
        self.measured_state = []
        self.states = []


        self.prev= rospy.Time.now().to_sec()

    def predict(self, u = 0):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        self.D = self.h/(self.h**2 + (self.d - self.x)**2) #update the D for extended kalman filter
        measurement = np.arctan(self.h/(self.d-self.x))
        if measurement < 0:
            measurement = np.pi - measurement
        y = z - measurement
        S = self.R + np.dot(self.D, np.dot(self.P, self.D.T))
        K = np.dot(np.dot(self.P, self.D.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(I - np.dot(K, self.D), self.P)


    def scan_call_back(self, data):
        self.phi = float(data.data)*math.pi/180
        print("get the angle data from the lidar")


    def run_kf(self,u):
        #kf = KalmanFilter(A=A, B=B, D=D, Q=Q, R=R)
        cur_measurement = self.phi
        dt = np.reshape(np.array([rospy.Time.now().to_sec() - self.prev]),(1,1))
        self.prev = rospy.Time.now().to_sec()
        print('dt:{}'.format(dt))
        self.B=dt
        

        self.measurements.append(cur_measurement)
        self.states.append(self.predict(u).flatten())
        self.update(cur_measurement)
        print("cur_pos:{}".format(self.x))
	





if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    cmd_publisher=rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    try:
        h = 0.6
        d = 0.6
        #dt = np.reshape(np.array([0.1]),(1,1))
        A = np.reshape(np.array([1]),(1,1))
        B = 0
        D = h/(h**2)        
        #D = np.reshape(np.array([h/(d**2)]),(1,1))
        Q = np.reshape(np.array([0.001]),(1,1)) #noise?
        R = np.reshape(np.array([0.001]),(1,1)) #noise?
        kf = KalmanFilter(A=A, B=B, D=D, Q=Q, R=R, h=h, d=d)
        while(1):
            key = getKey()
            twist=Twist()
#            if kf.x >= 1:
#                twist.linear.x=0
#                cmd_publisher.publish(twist)

            twist.linear.x=0.05
            cmd_publisher.publish(twist)
            kf.run_kf(0.05)  

            if (key == '\x03') or kf.x >=0.61:
                break
    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_publisher.publish(twist)

if __name__ == '__main__':
    main()


