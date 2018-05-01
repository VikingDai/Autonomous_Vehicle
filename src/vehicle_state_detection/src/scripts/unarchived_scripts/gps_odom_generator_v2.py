#!/usr/bin/env python
import rospy, math
import numpy as np
from utm import from_latlon
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from custom_msgs.msg import positionEstimate, orientationEstimate
from dbw_mkz_msgs.msg import WheelSpeedReport


class PoseDataCollector():

	def __init__(self):

		rospy.init_node('gps_odom_generator_node', anonymous=True)

		rospy.Subscriber("/mti/filter/position", positionEstimate, self.gpsUpdate)
		rospy.Subscriber("/vehicle/gps/fix", NavSatFix, self.gps2Update)
		rospy.Subscriber("/mti/filter/orientation", orientationEstimate, self.odomOrientationUpdate)
		rospy.Subscriber("/vehicle/wheel_speed_report", WheelSpeedReport, self.odomPositionUpdate)
		rospy.Subscriber("/vehicle/twist", TwistStamped, self.odomTwistUpdate)


		# publishers for vehicle localization...
		self.pub_gps = rospy.Publisher("/mti/odom/fix", Odometry, queue_size=10)
		self.pub_gps2 = rospy.Publisher("/vehicle/odom/fix", Odometry, queue_size=10)
		self.pub_odom = rospy.Publisher("/vehicle/odom/data_raw", Odometry, queue_size=10)

		self.gps_data = self.initializeOdom()
		self.gps2_data = self.initializeOdom()
		self.odom_data = self.initializeOdom()

		self.latitude_datum = 42.275921
		self.longitude_datum = -71.798370
		self.easting_datum = from_latlon(self.latitude_datum, self.longitude_datum)[0]
		self.northing_datum = from_latlon(self.latitude_datum, self.longitude_datum)[1]		

		self.odom_x = 0
		self.odom_y = 0
		self.odomYaw = 0

		self.canBusTimeInterval = 0.01
		self.wheelRadius = 0.34036 #FIXME Need measurements!
		self.trackWidth = 1.57861

		self.positionReceived  = False
		self.orientationReceived = False
		self.twistReceived = False
		self.wheelSigReceived = False

		self.publishOdom()

	def publishOdom(self):
		while not rospy.is_shutdown():
			rospy.loginfo("waiting for Subscribers...")
			rospy.sleep(0.5)
			while not rospy.is_shutdown() and self.positionReceived and self.orientationReceived and self.twistReceived and self.wheelSigReceived:
				self.publishOdomMsg()
				rospy.sleep(0.02)

	# subscriber callback functions:
	def gpsUpdate(self, data):
		self.gps_data.header.stamp = rospy.Time.now()
		easting = from_latlon(data.latitude, data.longitude)[0]
		northing = from_latlon(data.latitude, data.longitude)[1]
		self.gps_data.pose.pose.position.x = easting - self.easting_datum
		self.gps_data.pose.pose.position.y = northing - self.northing_datum
		self.publishGPSMsg()
		self.initOdomPose(data.longitude, data.latitude)

	def gps2Update(self, data):
		self.gps2_data.header.stamp = rospy.Time.now()
		easting = from_latlon(data.latitude, data.longitude)[0]
		northing = from_latlon(data.latitude, data.longitude)[1]
		self.gps2_data.pose.pose.position.x = easting - self.easting_datum
		self.gps2_data.pose.pose.position.y = northing - self.northing_datum
		self.publishGPSMsg2()

	def initOdomPose(self, longitude, latitude):
		if not self.positionReceived:
			# determine the first starting point. Run only once.
			easting = from_latlon(latitude, longitude)[0]
			northing = from_latlon(latitude, longitude)[1]
			self.odom_x = easting - self.easting_datum
			self.odom_y = northing - self.northing_datum
			self.positionReceived = True

	def odomPositionUpdate(self, data):
		self.wheelSigReceived = True
		speed = self.wheelRadius*(data.front_left + data.front_right + data.rear_left + data.rear_right)/4
		self.odom_x = self.odom_x + speed*self.canBusTimeInterval*math.cos(self.odomYaw)
		self.odom_y = self.odom_y + speed*self.canBusTimeInterval*math.sin(self.odomYaw)
		self.odom_data.header.stamp = rospy.Time.now()
		self.odom_data.pose.pose.position.x = self.odom_x
		self.odom_data.pose.pose.position.y = self.odom_y
		self.odom_data.pose.pose.position.z = 0

	def odomOrientationUpdate(self, data):
		self.orientationReceived = True
		self.odom_data.header.stamp = rospy.Time.now()
		# roll = math.radians(data.roll)
		# pitch = math.radians(data.pitch)
		self.odomYaw = math.radians(data.yaw)
		quaternion = quaternion_from_euler(0, 0, self.odomYaw)
		self.odom_data.pose.pose.orientation.x = quaternion[0]
		self.odom_data.pose.pose.orientation.y = quaternion[1]
		self.odom_data.pose.pose.orientation.z = quaternion[2]
		self.odom_data.pose.pose.orientation.w = quaternion[3]

	def odomTwistUpdate(self, data):
		self.twistReceived = True
		self.odom_data.header.stamp = rospy.Time.now()
		self.odom_data.twist.twist = data.twist

	# publishing functions
	def publishGPSMsg(self):
		self.pub_gps.publish(self.gps_data)

	def publishGPSMsg2(self):
		self.pub_gps2.publish(self.gps2_data)

	def publishOdomMsg(self):
		self.pub_odom.publish(self.odom_data)

	# initialize the publishing messages:
	def initializeGPS(self):
		msg = NavSatFix()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "odom"
		msg.position_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
		return msg

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
	PoseDataCollector()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
