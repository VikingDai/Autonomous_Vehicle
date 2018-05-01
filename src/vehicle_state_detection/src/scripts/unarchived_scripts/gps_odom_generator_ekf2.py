#!/usr/bin/env python
import rospy, math
import numpy as np
from utm import from_latlon
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

class GpsOdomGenerator():

	def __init__(self):

		rospy.init_node('ekf2_gps_odom_converter_node', anonymous=True)

		rospy.Subscriber("/gps/filtered_odom_mti", NavSatFix, self.updateFilteredPos_mti)
		rospy.Subscriber("/gps/filtered_odom_vehicle", NavSatFix, self.updateFilteredPos_vehicle)

		# publishers for vehicle localization...
		self.pub_odom_mti = rospy.Publisher("/odometry/filtered_odom_mti_2", Odometry, queue_size=10)
		self.pub_odom_vehicle = rospy.Publisher("/odometry/filtered_odom_vehicle_2", Odometry, queue_size=10)

		self.odom_mti_data = self.initializeOdom()
		self.odom_vehicle_data = self.initializeOdom()

		self.latitude_datum = 42.275921
		self.longitude_datum = -71.798370
		self.easting_datum = from_latlon(self.latitude_datum, self.longitude_datum)[0]
		self.northing_datum = from_latlon(self.latitude_datum, self.longitude_datum)[1]

	# subscriber callback functions:
	def updateFilteredPos_mti(self, data):
		self.odom_mti_data.header.stamp = data.header.stamp
		self.odom_mti_data.header.frame_id = "odom"
		self.odom_mti_data.child_frame_id = ""
		easting = from_latlon(data.latitude, data.longitude)[0]
		northing = from_latlon(data.latitude, data.longitude)[1]
		self.odom_mti_data.pose.pose.position.x = easting-self.easting_datum
		self.odom_mti_data.pose.pose.position.y = northing-self.northing_datum
		self.odom_mti_data.pose.pose.position.z = 0
		c = data.position_covariance
		self.odom_mti_data.pose.covariance = [c[0],c[1],c[2], 0, 0, 0,
                                              c[3],c[4],c[5], 0, 0, 0,
                                              c[6],c[7],c[8], 0, 0, 0,
                                              0   ,0   ,0   , 0, 0, 0,
                                              0   ,0   ,0   , 0, 0, 0,
                                              0   ,0   ,0   , 0, 0, 0]

		self.pub_odom_mti.publish(self.odom_mti_data)


	def updateFilteredPos_vehicle(self, data):
		self.odom_vehicle_data.header.stamp = data.header.stamp
		self.odom_vehicle_data.header.frame_id = "odom"
		self.odom_vehicle_data.child_frame_id = ""
		easting = from_latlon(data.latitude, data.longitude)[0]
		northing = from_latlon(data.latitude, data.longitude)[1]
		self.odom_vehicle_data.pose.pose.position.x = easting-self.easting_datum
		self.odom_vehicle_data.pose.pose.position.y = northing-self.northing_datum
		self.odom_vehicle_data.pose.pose.position.z = 0
		c = data.position_covariance
		self.odom_vehicle_data.pose.covariance = [c[0],c[1],c[2], 0, 0, 0,
                                                  c[3],c[4],c[5], 0, 0, 0,
                                                  c[6],c[7],c[8], 0, 0, 0,
                                                  0   ,0   ,0   , 0, 0, 0,
                                                  0   ,0   ,0   , 0, 0, 0,
                                                  0   ,0   ,0   , 0, 0, 0]

		self.pub_odom_vehicle.publish(self.odom_vehicle_data)

	# initialize the publishing messages:
	def initializeOdom(self):
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "odom"
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
	GpsOdomGenerator()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass