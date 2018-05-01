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

		rospy.init_node('gps_odom_generator_rvizTester', anonymous=True)

		rospy.Subscriber("/mti/gps/fix", NavSatFix, self.updateRawGPS)
		rospy.Subscriber("/vehicle/odom/data_raw", Odometry, self.updateRawOdom)
		rospy.Subscriber("/odometry/filtered_odom", Odometry, self.updateFilteredOdom)

		rospy.Subscriber("/odometry/filtered_odom_mti", Odometry, self.updateFilteredGPS1)
		rospy.Subscriber("/odometry/filtered_odom_vehicle", Odometry, self.updateFilteredGPS2)

		rospy.Subscriber("/odometry/filtered_final", Odometry, self.updateOdomFinal)

		# publishers for vehicle localization...
		self.pub_rviz_raw_gps = rospy.Publisher("/vehicle/rviz/gps_pose", PointStamped, queue_size=1)
		self.pub_rviz_raw_odom = rospy.Publisher("/vehicle/rviz/odom_pose", PointStamped, queue_size=1)
		self.pub_rviz_filtered_odom = rospy.Publisher("/vehicle/rviz/out_odom", PointStamped, queue_size=1)
		self.pub_rviz_filtered_gps1 = rospy.Publisher("/vehicle/rviz/out_gps1", PointStamped, queue_size=1)
		self.pub_rviz_filtered_gps2 = rospy.Publisher("/vehicle/rviz/out_gps2", PointStamped, queue_size=1)
		self.pub_rviz_filtered_final = rospy.Publisher("/vehicle/rviz/out_final", PointStamped, queue_size=1)

		self.rviz_gps_data = PointStamped()
		self.rviz_odom_data = PointStamped()
		self.rviz_out_gps1_data = PointStamped()
		self.rviz_out_gps2_data = PointStamped()
		self.rviz_out_odom_data = PointStamped()
		self.rviz_out_final_data = PointStamped()

		self.latitude_datum = 42.275921
		self.longitude_datum = -71.798370
		self.easting_datum = from_latlon(self.latitude_datum, self.longitude_datum)[0]
		self.northing_datum = from_latlon(self.latitude_datum, self.longitude_datum)[1]	


	def updateRawGPS(self, data):
		self.rviz_gps_data.header.stamp = rospy.Time.now()
		self.rviz_gps_data.header.frame_id = "map"
		easting = from_latlon(data.latitude, data.longitude)[0]
		northing = from_latlon(data.latitude, data.longitude)[1]
		self.rviz_gps_data.point.x = easting-self.easting_datum
		self.rviz_gps_data.point.y = northing-self.northing_datum
		self.rviz_gps_data.point.z = 0
		self.pub_rviz_raw_gps.publish(self.rviz_gps_data)

	def updateRawOdom(self, data):
		self.rviz_odom_data.header.stamp = rospy.Time.now()
		self.rviz_odom_data.header.frame_id = "map"
		self.rviz_odom_data.point.x = data.pose.pose.position.x
		self.rviz_odom_data.point.y = data.pose.pose.position.y
		self.rviz_odom_data.point.z = 0
		self.pub_rviz_raw_odom.publish(self.rviz_odom_data)

	def updateFilteredGPS1(self, data):
		self.rviz_out_gps1_data.header.stamp = rospy.Time.now()
		self.rviz_out_gps1_data.header.frame_id = "map"
		self.rviz_out_gps1_data.point.x = data.pose.pose.position.x
		self.rviz_out_gps1_data.point.y = data.pose.pose.position.y
		self.rviz_out_gps1_data.point.z = 0
		self.pub_rviz_filtered_gps1.publish(self.rviz_out_gps1_data)
		
	def updateFilteredGPS2(self, data):
		self.rviz_out_gps2_data.header.stamp = rospy.Time.now()
		self.rviz_out_gps2_data.header.frame_id = "map"
		self.rviz_out_gps2_data.point.x = data.pose.pose.position.x
		self.rviz_out_gps2_data.point.y = data.pose.pose.position.y
		self.rviz_out_gps2_data.point.z = 0
		self.pub_rviz_filtered_gps2.publish(self.rviz_out_gps2_data)

	def updateFilteredOdom(self, data):
		self.rviz_out_odom_data.header.stamp = rospy.Time.now()
		self.rviz_out_odom_data.header.frame_id = "map"
		self.rviz_out_odom_data.point.x = data.pose.pose.position.x
		self.rviz_out_odom_data.point.y = data.pose.pose.position.y
		self.rviz_out_odom_data.point.z = 0
		self.pub_rviz_filtered_odom.publish(self.rviz_out_odom_data)

	def updateOdomFinal(self, data):
		self.rviz_out_final_data.header.stamp = rospy.Time.now()
		self.rviz_out_final_data.header.frame_id = "map"
		self.rviz_out_final_data.point.x = data.pose.pose.position.x
		self.rviz_out_final_data.point.y = data.pose.pose.position.y
		self.rviz_out_final_data.point.z = 0
		self.pub_rviz_filtered_final.publish(self.rviz_out_final_data)		


def main():
	PoseDataCollector()
	rospy.spin()

if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
