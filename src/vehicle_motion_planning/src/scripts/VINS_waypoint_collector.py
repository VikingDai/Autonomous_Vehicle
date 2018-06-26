#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
import csv, os

#TODO: 1. capture the waypoints that was regulated by VINS.
#	   1.1. make judgement about when it's the time to save the data.
#      1.2. The resolution for the data can be customized.
#      2. save the waypoints as csv file.
#      3. update the waypointes as the waypoints got updated.

class WaypointCollector():

	def __init__(self):

		rospy.init_node('waypoint_collector', anonymous=True)

		rospy.Subscriber("/pose_graph/pose_graph_path", Path, self.collectPath)

		self.resolution = 1		# resolution between each waypoint (in meters)

		self.samplingStage = 1	# sampling stage, initial value is 1.

		self.desiredSpeed = 1	# speed in desired state.

		self.sequenceWaypointLength = 0

	def collectPath(self, data):
		# When to start collecting the path data and save it to csv:
		# Stage One: Loop closure (Before VINS starts optimizing.)
			# 1. End pose is closed enough with initial pose;
			# 2. The pose sequences was long enough -- (To avoid the situation that 
			#    the vehicle is just start estimating path and it satisfies the first condition).
		# Stage Two: When VINS starts to optimize after one loop is recorded:
			# 1. Keep adjusting the original waypoints.

		loop_gap_limit = 3
		waypoint_sequence_min = 100

		initPoseStamped = data.poses[0]
		endPoseStamped = data.poses[-1]

		if self.samplingStage == 1:

			sequence = len(data.poses)
			initX = initPoseStamped.pose.position.x
			initY = initPoseStamped.pose.position.y
			endX = endPoseStamped.pose.position.x
			endY = endPoseStamped.pose.position.y

			loop_gap = np.sqrt((endX-initX)**2 + (endY-initY)**2)
			if self.samplingStage == 1:
				if loop_gap <= loop_gap_limit and sequence > waypoint_sequence_min:
					rospy.loginfo("Found the loop! Path sequence: "+str(sequence)+" Gap: "+str(loop_gap))
					self.recordWaypoints(data.poses, self.resolution)
					self.samplingStage == 2
			else:
				self.recordWaypoints(data.poses, self.resolution)



	def recordWaypoints(self, posesList, resolution):

		lastPoseX = 0
		lastPoseY = 0
		lastSeq = 0
		accumulatedTime = 0

		# if the function is run in the first time, then update the waypoint length;
		# otherwise, keep the waypoint length every time it updates.
		if self.samplingStage == 1:
			self.sequenceWaypointLength = len(posesList)
		else:
			pass

		# target path is in dat folder in the previous directory.
		target_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ),"..","data"))+"/"
		file_name = 'dynamic_waypoints.csv'
		with open(str(target_path)+file_name, 'wb') as csvfile:

			filewriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			
			for i in range (self.sequenceWaypointLength):
				pose = posesList[i]
				seq = i
				xPose = pose.pose.position.x
				yPose = pose.pose.position.y
				if i == 0:						# the first waypoint: initialization
					lastPoseX = xPose
					lastPoseY = yPose
					accumulatedTime = 0
					lastSeq = seq
				else:
					deltaX = xPose - lastPoseX
					deltaY = yPose - lastPoseY
					dist = np.sqrt(deltaX**2 + deltaY**2)
					if dist > resolution:		# If the next waypoint distance is larger than the resolution, then record it.
						timeInterval = np.sqrt(deltaX**2+deltaY**2)/self.desiredSpeed
						accumulatedTime = accumulatedTime + timeInterval
						xdot = deltaX/timeInterval
						ydot = deltaY/timeInterval

						# encode it and write the initial waypoint info into .csv file
						dataInRow = [str(accumulatedTime), str(lastPoseX), str(lastPoseY), str(xdot), str(ydot), str(lastSeq)]
						filewriter.writerow(dataInRow)

						# update last waypoint position for next use
						lastPoseX = xPose
						lastPoseY = yPose
						lastSeq = seq
					else:
						pass					# If the next waypoint distance is less than the resolution, then discard it.












def main():
	WaypointCollector()
	rospy.spin()



if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
