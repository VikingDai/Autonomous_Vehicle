#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from custom_msgs.msg import positionEstimate

class MtiGpsRegulator():

	def __init__(self):

		rospy.init_node('mti_gps_regulator', anonymous=True)

		rospy.Subscriber("/mti/filter/position", positionEstimate, self.gpsUpdate)
		self.pub_gps = rospy.Publisher("/mti/gps/fix", NavSatFix, queue_size=10)

		self.gps_data = self.initializeGPS()

		rospy.loginfo('MTI GPS regulator start working.')


	# initialize the publishing messages:
	def initializeGPS(self):
		msg = NavSatFix()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"
		msg.position_covariance = [0, 0, 0,
								   0, 0, 0,
								   0, 0, 0]
		return msg

	# subscriber callback function:
	def gpsUpdate(self, data):
		self.gps_data.header.stamp = rospy.Time.now()
		self.gps_data.latitude = data.latitude
		self.gps_data.longitude = data.longitude
		self.gps_data.altitude = data.hEll
		self.publishGPSMsg()

	# publishing function
	def publishGPSMsg(self):
		self.pub_gps.publish(self.gps_data)


def main():
	MtiGpsRegulator()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass