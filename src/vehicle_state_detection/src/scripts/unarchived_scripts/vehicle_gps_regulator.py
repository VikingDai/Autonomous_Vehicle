#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix


class VehicleGpsRegulator():

	def __init__(self):

		rospy.init_node('vehicle_gps_regulator', anonymous=True)

		rospy.Subscriber("/vehicle/gps/fix", NavSatFix, self.gps2Update)
		self.pub_gps2 = rospy.Publisher("/vehicle/gps/fix2", NavSatFix, queue_size=10)

		self.gps2_data = self.initializeGPS()

		rospy.loginfo('CAN GPS regulator start working.')


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
	def gps2Update(self, data):
		self.gps2_data.header.stamp = rospy.Time.now()
		self.gps2_data.latitude = data.latitude
		self.gps2_data.longitude = data.longitude
		self.gps2_data.altitude = data.altitude
		self.publishGPSMsg2()

	# publishing function
	def publishGPSMsg2(self):
		self.pub_gps2.publish(self.gps2_data)


def main():
	VehicleGpsRegulator()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
