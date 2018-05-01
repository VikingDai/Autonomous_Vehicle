#!/usr/bin/env python
import rospy, math, time
import numpy as np
from control import lqr
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TwistStamped, PoseStamped
from dbw_mkz_msgs.msg import SteeringCmd, ThrottleCmd, BrakeCmd
from path_planning_fcn import PathPlanningModule

class VehicleController:

	def __init__(self):

		rospy.init_node('vehicle_control_node', anonymous=True)
		
		# Setup publishers and subscribers
		rospy.Subscriber("/vehicle/MTiPose", PoseStamped, self.poseUpdate)
		rospy.Subscriber("/vehicle/twist", TwistStamped, self.velUpdate)
		self.steering_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)


		self.startingTime = time.time()

		# create a new PathPlanningModule, and the waypoint list is loaded.
		self.pathModule = PathPlanningModule()

		# define the car's instant state with the format: [t, x, y, dx, dy]
		self.vehicleState = [0, 0, 0, 0, 0]
		self.linVel = 0
		self.maxVel = 4
		self.maxDelta = np.pi/10.4
		self.steerDeltaRatio = 14.9
		self.wheelbase = 2.84988

		self.degree2meter = 113119.48381195657

		self.refLat = 42.2757474
		self.refLong = -71.7983398


		self.rate = 1
		self.frequency = 5

		self.poseReceived  = False
		self.velReceived = False

		# auto-running functions:
		# initialization: waiting for messages from GNSS and Odometry to be subsribed/
		self.waitingForStart()
		# when the message received, run the controller.
		while not rospy.is_shutdown():
			self.controller()


	def waitingForStart(self):
		while not rospy.is_shutdown() and not (self.poseReceived and self.velReceived):
			if not (self.poseReceived and self.velReceived):
				rospy.loginfo('waiting for GNSS and Odom message to be published...')
			elif self.poseReceived and not self.velReceived:
				rospy.loginfo('GNSS received. waiting for Odom message to be published...')
			elif not self.poseReceived and self.velReceived:
				rospy.loginfo('Odom received. waiting for GNSS message to be published...')
			rospy.sleep(0.8)
		rospy.loginfo('GNSS and Odom message received. Control starts.')

	# Callback functions:
	def poseUpdate(self, data):
		longitude = data.pose.position.x
		latitude = data.pose.position.y
		# self.updateStateTime()
		self.vehicleState[1] = (longitude-self.refLong)*self.degree2meter
		self.vehicleState[2] = (latitude-self.refLat)*self.degree2meter
		quaternion = (data.pose.orientation.x,
					  data.pose.orientation.y, 
					  data.pose.orientation.z,
					  data.pose.orientation.w,)
		[roll, pitch, yaw] = euler_from_quaternion(quaternion)
		self.vehicleState[3] = self.linVel*math.cos(yaw)
		self.vehicleState[4] = self.linVel*math.sin(yaw)
		self.poseReceived = True

	def velUpdate(self, data):
		self.linVel = data.twist.linear.x
		self.velReceived = True

	# generate trajectories
	def getTrajectories(self):
		x = self.vehicleState[1]
		y = self.vehicleState[2]
		dx = self.vehicleState[3]
		dy = self.vehicleState[4]
		trajectories = self.pathModule.generateTrajectories(x,y,dx,dy)
		print(x,y)
		return trajectories

	def controller(self):
		# firstly, get segents of trajectories based on starting point.
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
				x = self.vehicleState[1]
				y = self.vehicleState[2]
				theta = math.atan2(self.vehicleState[4], self.vehicleState[3])
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

				thetades = math.atan2(dydes, dxdes)

				# desired state
				desiredState = np.array([[xdes],[ydes],[thetades]])

				# compute the feedforward in the input
				vf = dxdes*np.cos(thetades) + dydes*np.sin(thetades);
				dthetades = 1/vf*(ddydes*np.cos(thetades) - ddxdes*np.sin(thetades));
				desdelta = math.atan2(self.wheelbase*dthetades,vf);

				# set the dynamics
				A = np.array([
					[0, 0, -vf*np.sin(thetades)],
					[0, 0, vf*np.cos(thetades)],
					[0, 0, 0]
					])

				B = np.array([
					[np.cos(thetades),   0],
					[np.sin(thetades),   0],
					[np.tan(desdelta)/self.wheelbase, vf/self.wheelbase*(np.tan(desdelta)**2+1)]
					])

				Q = np.array([[1],[1],[1]])*np.eye(3)
				R = np.array([[1],[1]])*np.eye(2)

				K, S, E = lqr(A, B, Q, R)


				errorState = desiredState - state
				errorState[2] = self.wrapToPi(errorState[2])
				
				# print(np.dot(-1*K, errorState))

				u = np.dot(-1*K, errorState) + np.array([[vf],[desdelta]])
				u[0] = self.limitRange(u[0], self.maxVel, 0);
				u[1] = self.limitRange(u[1], self.maxDelta, -1*self.maxDelta)

				self.throttleBreakCtrl(u[0][0])
				self.steeringCtrl(u[1][0])

				r.sleep()

	def throttleBreakCtrl(self, desiredVel):
		vel_feedback = self.linVel
		errorVel = desiredVel - vel_feedback
		if errorVel > 0:
			throttle_pwr = 0.01*errorVel
			self.publishThrottleCmd(ThrottleCmd)
			self.publishBrakeCmd(0)
			print(throttle_pwr, 0)

		elif errorVel < -0.3:
			brake_pwr = 0.005*errorVel
			self.publishThrottleCmd(0)
			self.publishBrakeCmd(brake_pwr)
			print(0, brake_pwr)

	def steeringCtrl(self, desiredDeltaAngle):
		steer_angle = desiredDeltaAngle*self.steerDeltaRatio
		self.publishSteeringCmd(steer_angle)
		print(steer_angle)

	def wrapToPi(self, radNum):
		return np.mod(radNum+np.pi, 2*np.pi)-np.pi

	def limitRange(self, num, maxVal, minVal):
		if num >= maxVal:
			return maxVal
		elif num <= minVal:
			return minVal
		else:
			return num

	def publishThrottleCmd(self, pedalVal):
		msg = ThrottleCmd()
		msg.pedal_cmd = pedalVal
		msg.pedal_cmd_type = 2
		msg.enable = True
		msg.clear = False
		msg.ignore = False
		msg.count = 0
		self.throttle_pub.publish(msg)

	def publishBrakeCmd(self, pedalVal):
		msg = BrakeCmd()
		msg.pedal_cmd = pedalVal
		msg.pedal_cmd_type = 2
		msg.boo_cmd = False
		msg.enable = True
		msg.clear = False
		msg.ignore = False
		msg.count = 0
		self.brake_pub.publish(msg)

	def publishSteeringCmd(self, steerVal):
		msg = SteeringCmd()
		msg.steering_wheel_angle_cmd = steerVal
		msg.steering_wheel_angle_velocity = 0
		msg.enable = True
		msg.clear = False
		msg.ignore = False
		msg.quiet = False
		msg.count = 0
		self.steering_pub.publish(msg)



def main():
	VehicleController()
	rospy.spin()


if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass




