#!/usr/bin/env python
import rospy, math
from utm import from_latlon
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix, Imu
from dbw_mkz_msgs.msg import WheelSpeedReport


class OdomGenerator():

	def __init__(self):

		rospy.init_node('odom_generator', anonymous=True)

		self.sub_datum = rospy.Subscriber("/mti/sensor/fix", NavSatFix, self.datumUpdate)
		rospy.Subscriber("/mti/sensor/imu", Imu, self.odomOrientationUpdate)
		rospy.Subscriber("/vehicle/wheel_speed_report", WheelSpeedReport, self.odomPositionUpdate)
		rospy.Subscriber("/vehicle/twist", TwistStamped, self.odomTwistUpdate)

		# publishers for vehicle localization...
		self.pub_odom = rospy.Publisher("/vehicle/odom/data_raw", Odometry, queue_size=10)

		self.odom_data = self.initializeOdom()

		self.latitude_datum = 42.275921
		self.longitude_datum = -71.798370
		self.easting_datum = from_latlon(self.latitude_datum, self.longitude_datum)[0]
		self.northing_datum = from_latlon(self.latitude_datum, self.longitude_datum)[1]		

		self.odom_x = 0
		self.odom_y = 0
		self.odomYaw = 0

		self.canBusTimeInterval = 0.01
		self.wheelRadius = 0.34036 #FIXME Need more accurate measurements!
		self.trackWidth = 1.57861

		self.datumReceived  = False
		self.orientationReceived = False
		self.twistReceived = False
		self.wheelSigReceived = False

		self.publishOdom()

	def publishOdom(self):
		while not rospy.is_shutdown():
			rospy.loginfo("waiting for Subscribers...")
			# print(self.datumReceived, self.orientationReceived, self.twistReceived, self.wheelSigReceived)
			rospy.sleep(0.5)
			while not rospy.is_shutdown() and self.datumReceived and self.orientationReceived and self.twistReceived and self.wheelSigReceived:
				self.publishOdomMsg()
				rospy.sleep(0.02)

	# subscriber callback functions:
	def datumUpdate(self, data):
		if not self.datumReceived:
			# determine the first starting point. Run only once.
			easting = from_latlon(data.latitude, data.longitude)[0]
			northing = from_latlon(data.latitude, data.longitude)[1]
			self.odom_x = easting-self.easting_datum
			self.odom_y = northing-self.northing_datum
			self.datumReceived = True
			self.sub_datum.unregister()

	def odomPositionUpdate(self, data):
		self.wheelSigReceived = True

		if data.rear_left == data.rear_right or (data.rear_right == 0 and data.rear_left == 0):
			dist = self.wheelRadius*data.rear_left*self.canBusTimeInterval
			self.odom_x = self.odom_x + dist*math.cos(self.odomYaw)
			self.odom_y = self.odom_y + dist*math.sin(self.odomYaw)

		else:
			left_wheel_dist = self.wheelRadius*data.rear_left*self.canBusTimeInterval
			right_wheel_dist = self.wheelRadius*data.rear_right*self.canBusTimeInterval

			if data.rear_left > data.rear_right:
				inner_radius = self.trackWidth*right_wheel_dist / (left_wheel_dist - right_wheel_dist)
				inner_radius = inner_radius + 0.00000001
				theta_rad = right_wheel_dist / inner_radius
				delta_x = (1-math.cos(theta_rad))*(self.trackWidth/2 + inner_radius)
				delta_y = math.sin(theta_rad)*(self.trackWidth/2 + inner_radius)
			else:
				inner_radius = self.trackWidth*left_wheel_dist / (right_wheel_dist - left_wheel_dist)
				inner_radius = inner_radius + 0.00000001
				theta_rad = left_wheel_dist / inner_radius
				delta_x = -1*(1-math.cos(theta_rad))*(self.trackWidth/2 + inner_radius)
				delta_y = math.sin(theta_rad)*(self.trackWidth/2 + inner_radius)
			#coordinate transformation:
			turning_rad = math.atan2(delta_y, delta_x)
			movement_length = math.sqrt(delta_x**2 + delta_y**2)
			newYaw = self.wrapToPi(self.odomYaw + turning_rad)
			self.odom_x = self.odom_x + movement_length*math.cos(newYaw)
			self.odom_y = self.odom_y + movement_length*math.sin(newYaw)
		
		self.odom_data.header.stamp = rospy.Time.now()
		self.odom_data.pose.pose.position.x = self.odom_x
		self.odom_data.pose.pose.position.y = self.odom_y
		self.odom_data.pose.pose.position.z = 0

	def odomOrientationUpdate(self, data):
		self.orientationReceived = True
		self.odom_data.header.stamp = rospy.Time.now()
		self.odom_data.pose.pose.orientation.x = data.orientation.x
		self.odom_data.pose.pose.orientation.y = data.orientation.y
		self.odom_data.pose.pose.orientation.z = data.orientation.z
		self.odom_data.pose.pose.orientation.w = data.orientation.w

	def odomTwistUpdate(self, data):
		self.twistReceived = True
		self.odom_data.header.stamp = rospy.Time.now()
		self.odom_data.twist.twist = data.twist

	# publishing functions
	def publishOdomMsg(self):
		self.pub_odom.publish(self.odom_data)

	# initialize the publishing messages:
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

	# wrap the radian value to the range [-pi, pi]
	def wrapToPi(self, radNum):
		return math.fmod(radNum+math.pi, 2*math.pi)-math.pi

def main():
	OdomGenerator()
	rospy.spin()


if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
