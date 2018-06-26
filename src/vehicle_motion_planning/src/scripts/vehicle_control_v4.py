#!/usr/bin/env python
import rospy, math, time
import numpy as np
from control import lqr
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from dbw_mkz_msgs.msg import SteeringCmd, ThrottleCmd, BrakeCmd, GearCmd
from path_planning_fcn import PathPlanningModule
from pid import PID

class Gearbox():
	NONE, PARK, REVERSE, NEUTRAL, DRIVE, LOW = range(6)

class VehicleController:

	def __init__(self):

		rospy.init_node('vehicle_control_node', anonymous=True)
		
		# Setup publishers and subscribers
		rospy.Subscriber("/odometry/filtered_out", Odometry, self.poseUpdate)
		rospy.Subscriber("/mti/sensor/velocity", TwistStamped, self.speedUpdate)

		self.steering_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.gear_pub = rospy.Publisher('/vehicle/gear_cmd', GearCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

		# Rviz publisher
		self.currentPose_pub = rospy.Publisher('/vehicle/rviz/current_pose', PoseStamped, queue_size=1)
		self.desiredPose_pub = rospy.Publisher('/vehicle/rviz/desired_pose', PoseStamped, queue_size=1)

		self.startingTime = time.time()

		# create a new PathPlanningModule, and the waypoint list is loaded.
		self.pathModule = PathPlanningModule()

		# define the car's instant state with the format: [x, y, dx, dy]
		self.vehicleState = [0, 0, 0, 0]
		# define the car's desired state with the format: [x, y, dx, dy]
		self.desired_vehicleState = [0, 0, 0, 0]


		self.linVel = 0
		self.maxVel = 3.5
		self.maxDelta = 2.25*np.pi/14.9
		self.steerDeltaRatio = -14.9
		self.wheelbase = 2.84988

		self.throttle_PadalOffset = 0.03
		self.brake_PadalOffset = 0.12

		self.rate = 1
		self.frequency = 100

		self.poseReceived = False
		self.maxBrakeTorque = 3250

		# coefficients of PID
		self.pidCtrller = PID(2, 0.00, 0)

		# auto-running function:
		self.mainProgram()



	def mainProgram(self):
		# Event handler: register rospy shutdown event.
		rospy.on_shutdown(self.shutdownStop)
		self.waitingForStart()
		self.switchGearToDrive()
		# when the message received, run the controller.
		while not rospy.is_shutdown():
			self.LqrController()


	# initialization: waiting for messages from GNSS and Odometry to be subsribed
	def waitingForStart(self):
		while not rospy.is_shutdown() and not self.poseReceived:
			rospy.loginfo('Waiting for pose to update...')
			rospy.sleep(0.5)
		if not rospy.is_shutdown():
			rospy.loginfo('GNSS and Odom message received. The vehicle starts in 3 seconds...')

	# Switch gearbox from parking to drive
	def switchGearToDrive(self):
		startingTime = time.time()
		while not rospy.is_shutdown() and (time.time()-startingTime < 3):
			self.publishGearCmd(Gearbox.DRIVE)
			self.publishBrakeCmd(0.8*self.maxBrakeTorque)
			
	# Callback function of subscribers:
	def poseUpdate(self, data):
		self.vehicleState[0] = data.pose.pose.position.x
		self.vehicleState[1] = data.pose.pose.position.y
		quaternion = (data.pose.pose.orientation.x,
					  data.pose.pose.orientation.y,
					  data.pose.pose.orientation.z,
					  data.pose.pose.orientation.w)
		[roll, pitch, yaw] = euler_from_quaternion(quaternion)
		# self.linVel = data.twist.twist.linear.x
		self.vehicleState[2] = self.linVel*math.cos(yaw)
		self.vehicleState[3] = self.linVel*math.sin(yaw)
		self.poseReceived = True

	def speedUpdate(self, data):
		self.linVel = np.sqrt(data.twist.linear.x**2 + data.twist.linear.y**2 + data.twist.linear.z**2)

	# Generate trajectories:
	def getTrajectories(self):
		x = self.vehicleState[0]
		y = self.vehicleState[1]
		dx = self.vehicleState[2]
		dy = self.vehicleState[3]
		trajectories = self.pathModule.generateTrajectories(x,y,dx,dy)
		print(x,y)
		return trajectories

	def LqrController(self):
		# first, get segments of trajectories based on starting point.
		trajectories = self.getTrajectories()
		
		# runnig frequency of the controller.
		r = rospy.Rate(self.frequency)

		# The controller will follow the list of trajectories.
		for trajectory in trajectories:

			# reset the starting time when the vehicle enters a new trajectory segment
			self.startingTime = time.time()

			# read the estimated time needed for running through this trajectory
			timeInterval = trajectory[0]

			# read the coefficients of the trajectory expression in x and y axis
			xvec = trajectory[1:7]
			yvec = trajectory[7:13]

			# main controller.
			while not rospy.is_shutdown() and time.time()-self.startingTime < timeInterval:
				# get the instant vehicle state
				x = self.vehicleState[0]
				y = self.vehicleState[1]
				theta = math.atan2(self.vehicleState[3], self.vehicleState[2])
				state = np.array([[x], [y], [theta]])

				t = time.time()-self.startingTime
				basis = np.array([[1],[t],[t**2],[t**3],[t**4],[t**5]])
				dbasis = np.array([[0],[1],[2*t],[3*t**2],[4*t**3],[5*t**4]])
				ddbasis = np.array([[0],[0],[2],[6*t],[12*t**2],[20*t**3]])

				xdes = np.dot(xvec,basis)[0]
				dxdes = np.dot(xvec,dbasis)[0]
				ddxdes = np.dot(xvec,ddbasis)[0]

				ydes = np.dot(yvec,basis)[0]
				dydes = np.dot(yvec,dbasis)[0]
				ddydes = np.dot(yvec,ddbasis)[0]

				# LQR cannot calculate a stable value when velocity is 0. Therefore, set a minimum value for dydes
				if abs(dydes) < 0.0001:
					dydes = 0.0001

				self.desired_vehicleState = [xdes, ydes, dxdes, dydes]

				thetades = math.atan2(dydes, dxdes)

				# desired state
				desiredState = np.array([[xdes],[ydes],[thetades]])

				# compute the feedforward in the input
				vf = dxdes*np.cos(thetades) + dydes*np.sin(thetades);
				dthetades = 1/vf*(ddydes*np.cos(thetades) - ddxdes*np.sin(thetades));
				deltades = math.atan2(self.wheelbase*dthetades,vf);

				# set the dynamics
				A = np.array([
					[0, 0, -vf*np.sin(thetades)],
					[0, 0, vf*np.cos(thetades)],
					[0, 0, 0]
					])

				B = np.array([
					[np.cos(thetades),   0],
					[np.sin(thetades),   0],
					[np.tan(deltades)/self.wheelbase, vf/self.wheelbase*(np.tan(deltades)**2+1)]
					])

				Q = np.array([[1],[1],[1]])*np.eye(3)
				R = np.array([[1],[1]])*np.eye(2)

				K, S, E = lqr(A, B, Q, R)

				errorState = state - desiredState
				errorState[2] = self.wrapToPi(errorState[2])
				# print("Error State: ["+str(np.float32(errorState[0][0]))+", "+str(np.float32(errorState[1][0]))+", "+str(np.float32(errorState[2][0]))+"]")
				
				u = np.dot(K, errorState) + np.array([[vf],[deltades]])
				u[0] = -1*self.limitRange(u[0], self.maxVel, -1*self.maxVel);
				u[1] = self.limitRange(u[1], self.maxDelta, -1*self.maxDelta)
				# print("Input velocity: "+str(u[0])+", Input steering andle: "+str(u[1]))
				# print(u[0][0])

				self.throttleBreakCtrl(u[0][0])
				# self.throttleBreakCtrl(1)
				self.steeringCtrl(u[1][0])

				self.publishCurrentPose()
				self.publishDesiredPose()

				r.sleep()

	# lower-level throttle/break and steering controller
	def throttleBreakCtrl(self, desiredVel):
		error = desiredVel - self.linVel
		print(self.linVel)
		output = self.pidCtrller.step(error, 0.02)
		if output > 0:
			throttleVal = output*0.01
			self.publishThrottleCmd(throttleVal)
			self.publishBrakeCmd(0)
			# print(self.linVel, throttleVal)
		else:
			brakeVal = -900*output
			self.publishThrottleCmd(0)
			self.publishBrakeCmd(brakeVal)
			# print(self.linVel, brakeVal)

	def steeringCtrl(self, desiredDeltaAngle):
		steer_angle = desiredDeltaAngle*self.steerDeltaRatio
		self.publishSteeringCmd(steer_angle)
		# print(steer_angle)

	# wrap the radian value to the range [-pi, pi]
	def wrapToPi(self, radNum):
		return np.mod(radNum+np.pi, 2*np.pi)-np.pi

	def limitRange(self, num, maxVal, minVal):
		if num >= maxVal:
			return maxVal
		elif num <= minVal:
			return minVal
		else:
			return num

	# callback event for rospy.on_shutdown() registration handler
	def shutdownStop(self):
		rospy.loginfo('Controller program terminated. The vehicle stops.')
		while abs(self.linVel) > 0.01:
			self.publishThrottleCmd(0)
			self.publishBrakeCmd(0.5*self.maxBrakeTorque)

		startingTime = time.time()
		while not rospy.is_shutdown() and (time.time()-startingTime < 3):
			self.publishGearCmd(Gearbox.PARK)
			self.publishBrakeCmd(0.8*self.maxBrakeTorque)


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


	# Rviz publishers:
	def publishCurrentPose(self):
		msg = self.generatePoseMsg(self.vehicleState)
		self.currentPose_pub.publish(msg)

	def publishDesiredPose(self):
		msg = self.generatePoseMsg(self.desired_vehicleState)
		self.desiredPose_pub.publish(msg)

	def generatePoseMsg(self, stateArray):
		msg = PoseStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "map"
		msg.pose.position.x = stateArray[0]
		msg.pose.position.y = stateArray[1]
		yaw = math.atan2(stateArray[3], stateArray[2])
		quaternion = quaternion_from_euler(0, 0, yaw)
		msg.pose.orientation.x = quaternion[0]
		msg.pose.orientation.y = quaternion[1]
		msg.pose.orientation.z = quaternion[2]
		msg.pose.orientation.w = quaternion[3]
		return msg


def main():
	VehicleController()
	rospy.spin()


if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass




