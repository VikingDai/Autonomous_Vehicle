#!/usr/bin/env python
import rospy, math
import numpy as np
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from custom_msgs.msg import positionEstimate, orientationEstimate
from dbw_mkz_msgs.msg import WheelSpeedReport


class PoseDataCollector():

	def __init__(self):

		rospy.init_node('pose_data_collector_with_odom', anonymous=True)

		# rospy.Subscriber("/mti/filter/position", positionEstimate, self.gpsUpdate)
		rospy.Subscriber("/vehicle/gps/fix", NavSatFix, self.gpsUpdate)
		rospy.Subscriber("/mti/filter/orientation", orientationEstimate, self.odomOrientationUpdate)
		# rospy.Subscriber("mti/sensor/imu", Imu, self.imuUpdate)
		rospy.Subscriber("/vehicle/imu/data_raw", Imu, self.imuUpdate)
		rospy.Subscriber("/vehicle/wheel_speed_report", WheelSpeedReport, self.odomPositionUpdate)
		rospy.Subscriber("/vehicle/twist", TwistStamped, self.odomTwistUpdate)
		rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.kalmanUpdate)

		self.pub_filtered_gps = rospy.Publisher("/gps", Odometry, queue_size=1)
		self.pub_filtered_imu = rospy.Publisher("/imu_data", Imu, queue_size=1)
		self.pub_filtered_odom = rospy.Publisher("/odom", Odometry, queue_size=1)
		# Rviz publishers...
		self.pub_rviz_gps = rospy.Publisher("/vehicle/rviz/gps_pose", PointStamped, queue_size=1)
		self.pub_rviz_odom = rospy.Publisher("/vehicle/rviz/odom_pose", PointStamped, queue_size=1)
		self.pub_rviz_kalman = rospy.Publisher("/vehicle/rviz/kalman_pose", PointStamped, queue_size=1)

		self.imu_data = Imu()
		self.gps_data = self.initializePose()
		self.odom_data = self.initializeOdom()

		self.rviz_gps_data = PointStamped()
		self.rviz_odom_data = PointStamped()
		self.rviz_kalman_data = PointStamped()


		self.degree2meter = 113119.48381195657
		self.refLat = 42.2757474
		self.refLong = -71.7983398
		self.odom_x = 0
		self.odom_y = 0
		self.yaw = 0

		self.canBusTimeInterval = 0.01
		self.wheelRadius = 0.34036 #FIXME Need measurements!
		self.trackWidth = 1.57861

		self.positionReceived  = False
		self.imuReceived = False
		self.orientationReceived = False
		self.twistReceived = False
		self.wheelSigReceived = False

		self.publishMsg()

	def publishMsg(self):
		while not rospy.is_shutdown():
			rospy.loginfo("waiting for Subscribers...")
			rospy.sleep(0.8)
			while not rospy.is_shutdown() and self.positionReceived and self.imuReceived and self.orientationReceived and self.twistReceived and self.wheelSigReceived:
				self.publishGPS()
				self.publishIMU()
				self.publishOdom()
				self.publishKalman()
				rospy.sleep(0.02)

	def publishGPS(self):
   		self.pub_filtered_gps.publish(self.gps_data)
   		self.pub_rviz_gps.publish(self.rviz_gps_data)

	def publishIMU(self):
		self.pub_filtered_imu.publish(self.imu_data)

	def publishOdom(self):
		self.pub_filtered_odom.publish(self.odom_data)
		self.pub_rviz_odom.publish(self.rviz_odom_data)

	def publishKalman(self):
		self.pub_rviz_kalman.publish(self.rviz_kalman_data)

	def kalmanUpdate(self, data):
		# rviz...
		self.rviz_kalman_data.header.stamp = rospy.Time.now()
		self.rviz_kalman_data.header.frame_id = "base_footprint"
		self.rviz_kalman_data.point.x = data.pose.pose.position.x
		self.rviz_kalman_data.point.y = data.pose.pose.position.y
		self.rviz_kalman_data.point.z = 0


	def gpsUpdate(self, data):
		longitude = data.longitude
		latitude = data.latitude
		x = (longitude-self.refLong)*self.degree2meter
		y = (latitude-self.refLat)*self.degree2meter
		if not self.positionReceived:
			# determine the first starting point. Run only once.
			self.odom_x = x
			self.odom_y = y
		self.positionReceived = True
		self.gps_data.header.stamp = rospy.Time.now()
		self.gps_data.pose.pose.position.x = x
		self.gps_data.pose.pose.position.y = y
		self.gps_data.pose.pose.position.z = 0
		
		# rviz...
		self.rviz_gps_data.header.stamp = rospy.Time.now()
		self.rviz_gps_data.header.frame_id = "base_footprint"
		self.rviz_gps_data.point.x = x
		self.rviz_gps_data.point.y = y
		self.rviz_gps_data.point.z = 0

	def odomOrientationUpdate(self, data):
		self.orientationReceived = True
		self.odom_data.header.stamp = rospy.Time.now()
		self.yaw = math.radians(data.yaw)
		quaternion = quaternion_from_euler(0, 0, self.yaw)
		self.odom_data.pose.pose.orientation.x = quaternion[0]
		self.odom_data.pose.pose.orientation.y = quaternion[1]
		self.odom_data.pose.pose.orientation.z = quaternion[2]
		self.odom_data.pose.pose.orientation.w = quaternion[3]

	def odomTwistUpdate(self, data):
		self.twistReceived = True
		self.odom_data.header.stamp = rospy.Time.now()
		self.odom_data.twist.twist = data.twist

	def odomPositionUpdate(self, data):
		x_gain = 1.35
		self.wheelSigReceived = True
		speed = self.wheelRadius*(data.front_left + data.front_right)/2
		self.odom_x = self.odom_x + speed*self.canBusTimeInterval*math.cos(self.yaw)*x_gain
		self.odom_y = self.odom_y + speed*self.canBusTimeInterval*math.sin(self.yaw)
		self.odom_data.header.stamp = rospy.Time.now()
		self.odom_data.pose.pose.position.x = self.odom_x
		self.odom_data.pose.pose.position.y = self.odom_y
		self.odom_data.pose.pose.position.z = 0

		# rviz...
		self.rviz_odom_data.header.stamp = rospy.Time.now()
		self.rviz_odom_data.header.frame_id = "base_footprint"
		self.rviz_odom_data.point.x = self.odom_x
		self.rviz_odom_data.point.y = self.odom_y
		self.rviz_odom_data.point.z = 0


	def imuUpdate(self, data):
		self.imuReceived = True
		self.imu_data.orientation = data.orientation
		self.imu_data.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.imu_data.angular_velocity = data.angular_velocity
		self.imu_data.angular_velocity_covariance = data.angular_velocity_covariance
		self.imu_data.linear_acceleration = data.linear_acceleration
		self.imu_data.linear_acceleration_covariance = data.linear_acceleration_covariance
		self.imu_data.header.stamp = rospy.Time.now()
		self.imu_data.orientation.w = 1
		self.imu_data.header.frame_id = "base_footprint"

	def initializeOdom(self):
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_footprint"
		msg.pose.pose.orientation.w = 1
		msg.pose.covariance = [1, 0, 0, 0, 0, 0,
							   0, 1, 0, 0, 0, 0,
							   0, 0, 1, 0, 0, 0,
							   0, 0, 0, 1, 0, 0,
							   0, 0, 0, 0, 1, 0,
							   0, 0, 0, 0, 0, 1]
		msg.twist.covariance = [1, 0, 0, 0, 0, 0,
							    0, 1, 0, 0, 0, 0,
							    0, 0, 1, 0, 0, 0,
							    0, 0, 0, 1, 0, 0,
							    0, 0, 0, 0, 1, 0,
							    0, 0, 0, 0, 0, 1]
		return msg

	def initializePose(self):
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_footprint"
		msg.pose.pose.orientation.w = 1
		msg.pose.covariance = [1, 0, 0, 0, 0, 0,
							   0, 1, 0, 0, 0, 0,
							   0, 0, 1, 0, 0, 0,
							   0, 0, 0, 99999, 0, 0,
							   0, 0, 0, 0, 99999, 0,
							   0, 0, 0, 0, 0, 99999]
		return msg

def main():
	PoseDataCollector()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
