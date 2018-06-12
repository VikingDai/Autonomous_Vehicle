#!/usr/bin/env python
import rospy, math, time
import numpy as np
from control import lqr
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from dbw_mkz_msgs.msg import SteeringCmd, ThrottleCmd, BrakeCmd, GearCmd
from custom_msgs.msg import positionEstimate, orientationEstimate

from pid import PID

import matplotlib.pyplot as plt


class Gearbox():
	NONE, PARK, REVERSE, NEUTRAL, DRIVE, LOW = range(6)

class DBWStatus():
	ENABLE, DISABLE = range(2)

class ThottleBrakeController:

	def __init__(self):

		rospy.init_node('tb_control_node', anonymous=True)
		
		# Setup publishers and subscribers
		rospy.Subscriber("/vehicle/twist", TwistStamped, self.speedUpdate)

		self.steering_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.gear_pub = rospy.Publisher('/vehicle/gear_cmd', GearCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
		self.dbw_enable = rospy.Publisher('/vehicle/enable', Empty, queue_size=1)
		self.dbw_disable = rospy.Publisher('/vehicle/disable', Empty, queue_size=1)


		self.startingTime = time.time()
		self.pidCtrller = PID(7, 0.00, 4)
		self.currentSpeed = 0
		self.maxBrakeTorque = 3250

		self.moveFlag = False

		self.runTimeStamp = 0

		plt.axis([0,10,0,5])

		self.mainProgram()


	def mainProgram(self):
		# Event handler: register rospy shutdown event.
		rospy.on_shutdown(self.shutdownStop)
		self.enableToStart()
		self.switchGearToDrive()

		# when the message received, run the controller.
		while not rospy.is_shutdown():
			self.mainController()
		plt.show()

	# run main controller in the system
	def mainController(self):
		self.thottleBrakeController()

	# initialization: waiting for messages from GNSS and Odometry to be subsribed
	def enableToStart(self):
		rospy.loginfo("enable dbw")
		self.publishDBWState(DBWStatus.ENABLE)
		rospy.sleep(0.1)

	# Switch gearbox from parking to drive
	def switchGearToDrive(self):
		rospy.loginfo("switching gear to drive")
		startingTime = time.time()
		while not rospy.is_shutdown() and (time.time()-startingTime < 3):
			self.publishGearCmd(Gearbox.DRIVE)
			self.publishBrakeCmd(self.maxBrakeTorque*0.8)

	# throttle/brake controller
	def thottleBrakeController(self):
		if not self.moveFlag:
			self.runTimeStamp = time.time()
			self.moveFlag = True

		desireSpeed = 1
		error = desireSpeed - self.currentSpeed
		output = self.pidCtrller.step(error, 0.02)
		if output > 0:
			throttleVal = output*0.01
			self.publishThrottleCmd(throttleVal)
			self.publishBrakeCmd(0)
			# print(self.currentSpeed, throttleVal)
		else:
			brakeVal = -900*output
			self.publishThrottleCmd(0)
			self.publishBrakeCmd(brakeVal)
			# print(self.currentSpeed, brakeVal)

		runTime = time.time() - self.runTimeStamp
		print(runTime, self.currentSpeed)
		plt.scatter(runTime, self.currentSpeed)


	# speed subscriber
	def speedUpdate(self, data):
		self.currentSpeed = data.twist.linear.x


	# Publishing functionsa
	def publishDBWState(self, status):
		msg = Empty()
		if status == DBWStatus.ENABLE:
			self.dbw_enable.publish(msg)
		elif status == DBWStatus.DISABLE:
			self.dbw_disable.publish(msg)
		else:
			rospy.logerr("Unknown DBW Status. Disable DBW in default.")
			self.dbw_disable.publish(msg)

	# throttle, break, gear and steering publishers
	def publishThrottleCmd(self, pedalVal):
		msg = ThrottleCmd()
		msg.pedal_cmd = pedalVal
		msg.pedal_cmd_type = 2
		'''
		pedal_cmd_type=1: Unitless, range 0.15 to 0.50
					  =2: Percent of maximum throttle, range 0 to 1
		'''
		msg.enable = True
		msg.clear = False
		msg.ignore = False
		msg.count = 0
		self.throttle_pub.publish(msg)

	def publishBrakeCmd(self, pedalVal):
		msg = BrakeCmd()
		msg.pedal_cmd = pedalVal
		msg.pedal_cmd_type = 3
		'''
		pedal_cmd_type=1: Unitless, range 0.15 to 0.50
					  =2: Percent of maximum torque, range 0 to 1
					  =3: Torque (unit: N.m), range 0 to 3250
		'''
		msg.boo_cmd = False
		msg.enable = True
		msg.clear = False
		msg.ignore = False
		msg.count = 0
		self.brake_pub.publish(msg)

	def publishSteeringCmd(self, steerVal):
		msg = SteeringCmd()
		msg.steering_wheel_angle_cmd = steerVal
		msg.steering_wheel_angle_velocity = 5 # 0-8.7, 0=maximum
		msg.enable = True
		msg.clear = False
		msg.ignore = False
		msg.quiet = False
		msg.count = 0
		self.steering_pub.publish(msg)

	def publishGearCmd(self, gear):
		msg = GearCmd()
		msg.cmd.gear = gear
		self.gear_pub.publish(msg)


	# callback event for rospy.on_shutdown() registration handler
	def shutdownStop(self):
		rospy.loginfo('Controller program terminated. The vehicle stops.')
		while abs(self.currentSpeed) > 0.01:
			self.publishThrottleCmd(0)
			self.publishBrakeCmd(self.maxBrakeTorque*0.5)

		startingTime = time.time()
		while not rospy.is_shutdown() and (time.time()-startingTime < 3):
			self.publishGearCmd(Gearbox.PARK)
			self.publishBrakeCmd(self.maxBrakeTorque*0.8)


def main():
	ThottleBrakeController()
	rospy.spin()


if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass