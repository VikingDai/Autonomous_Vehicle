#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from custom_msgs.msg import positionEstimate, orientationEstimate

global msg
global pub_path


class PoseDataCollector():

	def __init__(self):

		rospy.init_node('pose_data_collector', anonymous=True)
		rospy.Subscriber("/mti/filter/position", positionEstimate, self.positionUpdate)
		rospy.Subscriber("mti/sensor/imu", Imu, self.imuUpdate)
		self.pub_filtered_gps = rospy.Publisher("/gps", Odometry, queue_size=1)
		self.pub_filtered_imu = rospy.Publisher("/imu_data", Imu, queue_size=1)

		self.imu_data = 0

		# define the car's instant state with the format: [x, y, dx, dy]
		self.x = 0
		self.y = 0
		self.z = 0

		self.degree2meter = 113119.48381195657
		self.refLat = 42.2757474
		self.refLong = -71.7983398

		self.positionReceived  = False
		self.imuReceived = False
		self.velReceived = False

		self.publishMsg()

	def publishMsg(self):
		while not rospy.is_shutdown():
			rospy.loginfo("waiting for Subscribers...")
			rospy.sleep(0.8)
			while not rospy.is_shutdown() and self.positionReceived and self.imuReceived:
				self.publishGPS()
				self.publishIMU()
				rospy.sleep(0.03)


	def publishGPS(self):
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_footprint"
		msg.pose.pose.position.x = self.x
		msg.pose.pose.position.y = self.y
		msg.pose.pose.position.z = self.z
		msg.pose.pose.orientation.x = 1
		msg.pose.pose.orientation.y = 0
		msg.pose.pose.orientation.z = 0
		msg.pose.pose.orientation.w = 0
		msg.pose.covariance = [1, 0, 0, 0, 0, 0,
							   0, 1, 0, 0, 0, 0,
							   0, 0, 1, 0, 0, 0,
							   0, 0, 0, 99999, 0, 0,
							   0, 0, 0, 0, 99999, 0,
							   0, 0, 0, 0, 0, 99999]
   		self.pub_filtered_gps.publish(msg)

	def publishIMU(self):
		self.pub_filtered_imu.publish(self.imu_data)

	def positionUpdate(self, data):
		longitude = data.longitude
		latitude = data.latitude
		altitude = data.hEll
		self.x = (longitude-self.refLong)*self.degree2meter
		self.y = (latitude-self.refLat)*self.degree2meter
		self.z = altitude
		self.positionReceived = True

	def imuUpdate(self, data):
		self.imu_data = data
		self.imu_data.header.stamp = rospy.Time.now()
		self.imu_data.orientation.w = 1
		self.imu_data.header.frame_id = "base_footprint"
		self.imuReceived = True

def main():
	PoseDataCollector()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
