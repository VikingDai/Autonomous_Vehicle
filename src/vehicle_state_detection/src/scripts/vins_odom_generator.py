#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry

# A simple script to transform VINS output (nav_msgs/Path) to 
# current odometry info that can be fed into EKF in robot_localization package

class VINSPositionSampler():

	def __init__(self):

		rospy.init_node('VINS_position_sampler', anonymous=True)

		# Define the publisher(s) and the subscriber(s)
		rospy.Subscriber("/pose_graph/pose_graph_path", Path, self.updatePosition)
		self.odom_pub = rospy.Publisher('/vins/filtered_odom', Odometry, queue_size=1)

		self.odom_data = self.initializeOdom()

	def updatePosition(self, data):
		x = data.poses[-1].pose.position.x
		y = data.poses[-1].pose.position.y
		odom = self.odom_data
		odom.header.stamp = rospy.Time.now()
		odom.pose.pose.position.x = x
		odom.pose.pose.position.y = y
		self.odom_pub.publish(odom)

	def initializeOdom(self):
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "odom"
		msg.child_frame_id = "base_link"
		msg.pose.pose.orientation.w = 1
		msg.pose.covariance = [0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0]
		msg.twist.covariance = [0, 0, 0, 0, 0, 0,
							    0, 0, 0, 0, 0, 0,
							    0, 0, 0, 0, 0, 0,
							    0, 0, 0, 0, 0, 0,
							    0, 0, 0, 0, 0, 0,
							    0, 0, 0, 0, 0, 0]
		return msg

def main():
	VINSPositionSampler()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
