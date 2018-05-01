#!/usr/bin/env python
import rospy
import math, sys, os, datetime, csv

from nav_msgs.msg import Path
from std_msgs.msg import Header, Float32, Float64
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped, QuaternionStamped, Pose, PoseStamped, Point, Quaternion
# from gps_common.msg import GPSFix, GPSStatus
# from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class vehiclePosTester:

	def __init__(self,ns):
		self.ns = ns
		rospy.init_node('Position_Tester', anonymous=True)

		# Setup publishers and subscribers
		# the format(ns) looks for the namespace in the ros parameter server
		rospy.Subscriber("/vehicle/MTiPose", PoseStamped, self.poseUpdate)
		self.pub_path = rospy.Publisher('CarPath'.format(ns), Path, queue_size=10)
		self.pub_pose = rospy.Publisher('CarPos'.format(ns), PoseStamped, queue_size=10)

		# Start flage will mark a certain initialized function running only once.
		self.startFlag = True

		self.latitude = 0
		self.longitude = 0

		# Geography infor for calculating rate of latitude and longitude. Initialized as 0.
		self.latitudeOffset = 0
		self.longitudeOffset = 0
		self.latitudeToMeterRate = 0
		self.longitudeToMeterRate = 0

		# The vehicle's trajectory was defined as Path.
		self.traj_msg = Path()
		self.traj_msg.header.stamp = rospy.Time.now()
		self.traj_msg.header.frame_id = "map"

		# The vehicle's instant pose was defined as PoseStamped.
		self.pose_msg = PoseStamped();

		self.startdateNTime = 0
		self.startROSTime = 0

	def initPos(self, initLatitude, initLongitude):
		self.latitudeOffset = initLatitude
		self.longitudeOffset = initLongitude

	def geoPosToMeterRate(self, latitude):
		# rLat: reference latitude in radians
		rLat = latitude*math.pi/180
		self.latitudeToMeterRate = 111319.48381195657
		self.longitudeToMeterRate = 111319.48381195657 # *math.cos(rLat)

	def addPoseToPath(self, newPoseStamped):
		self.traj_msg.poses.append(newPoseStamped)

	def publishPath(self):
		self.pub_path.publish(self.traj_msg)

	def publishPose(self):
		self.pub_pose.publish(self.pose_msg)

	def poseUpdate(self, data):
		self.callbackMainProgram(data)

	# ********************** Recorder functions *******************************

	def configCSV(self):
		self.startdateNTime = str(datetime.datetime.now())
		target_path = "/home/ak209/catkin_ws/csv_data"
		with open(str(target_path) + "/" + self.startdateNTime + '.csv', 'wb') as csvfile:
			filewriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			initialMark = ['Time', '[Latitude, Longitute]']
			filewriter.writerow(initialMark)

	def writeData(self, longitude, latitude):
		target_path = "/home/ak209/catkin_ws/csv_data"
		with open(str(target_path) + "/" + self.startdateNTime + '.csv', 'a') as csvfile:
			filewriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			dataInRow = [str(rospy.Time.now() - self.startROSTime), str(latitude)+","+str(longitude)]
			filewriter.writerow(dataInRow)

	def callbackMainProgram(self, instPose):

		# decode the PoseStamped to latitude and longtitute, and assign them to attribute
		self.latitude = instPose.pose.position.x
		self.longitude = instPose.pose.position.y

		# Initialization. Runs once.
		if self.startFlag == True:
			self.initPos(self.latitude, self.longitude)
			self.configCSV()
			print("Initial position received: "+"["+str(self.latitude)+","+str(self.longitude)+"]")
			self.startROSTime = rospy.Time.now()
			self.startFlag = False
		
		# update Lat/Long2Meter's rate
		self.geoPosToMeterRate(self.latitude)

		# upadte converted position message into the pose_message attribute
		self.pose_msg = self.convertGeoMsgToMeter(instPose)

		# add pose to path, and publish path.
		self.addPoseToPath(self.pose_msg)
		self.publishPath()

		# publish pose
		self.publishPose()

		# write geo data to the csv file
		self.writeData(self.latitude, self.longitude)


	def convertGeoMsgToMeter(self, instPose):
		instPose.pose.position.x = (self.latitude - self.latitudeOffset)*self.latitudeToMeterRate
		instPose.pose.position.y = (self.longitude - self.longitudeOffset)*self.longitudeToMeterRate
		instPose.pose.position.z = 0
		instPose.header.stamp = rospy.Time.now()
		instPose.header.frame_id = "map"
		return instPose



def main(argv):

	# name space define
	ns = sys.argv[1]
	node = vehiclePosTester(ns)
	rospy.spin()


if __name__== '__main__':
	try:
		main(sys.argv)
	except rospy.ROSInterruptException:
		pass

