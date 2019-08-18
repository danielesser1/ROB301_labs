
import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def camera_callback(data):
	bridge = CvBridge()
	try:
		cv_img = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
		print(e)

	array = opencv_image_as_array(cv_img)
	mid = len(array)//2
	array = array[mid-10: mid+10]
	array = np.mean(array, axis=1)

	index = np.max(array)
	rospy.loginfo(index)
	rospy.sleep(0.01)
	return


def main():
	rospy.init_node('camera_rgb')
	camera_subscriber = rospy.Subscriber('raspicam_node/image', Image, camera_callback, queue_size = 1)
	rospy.spin()


if __name__ == '__main__':
	main()