#!/usr/bin/env python

#TODO: load the stamped trajectory. -Done
#TODO: unify the unit to meters. -Done
#TODO: subscribe instant vehicle state, and feasible trajectory for the vehicle. -Done
#TODO: initially, find the closest waypoint and generate the trajectory based on its following waypoints. -Done
#TODO: traj refresh rate needed - NOT Done.
#TODO: how to determine when to update traj...  -Done

import rospy, math, time, os
import numpy as np
from numpy import genfromtxt
from generate_traj import generate_traj

class PathPlanningModule:

	def __init__(self):

		# define the car's instant state with the format: [t, x, y, dx, dy]
		self.wayPointList = []
		# psudo velocity set as 1m/s.
		self.psudoVel = 0.6
		self.trajNum = 100;
		self.dataFileName = 'coarseState_1.csv'

		self.x_offset = 0
		self.y_offset = -3
		self.readWaypoints();

	def generateTrajectories(self, x, y, dx, dy):
		vehicleState = [0, x, y, dx, dy]
		startingIndex = self.startingWaypointIndex(vehicleState)
		trajVectorList = self.waypointToTraj(startingIndex, vehicleState)
		return trajVectorList


	# support functions
	def readWaypoints(self):
		# read the raw waypoints from the .csv file
		target_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ),"..","data"))+"/"
		self.wayPointList = genfromtxt(target_path+self.dataFileName, delimiter=',')
		for i in range (len(self.wayPointList)-1,-1,-1):
			if i == 0:
				nextX = self.wayPointList[-1][1]
				nextY = self.wayPointList[-1][2]
				thisX = self.wayPointList[i][1]
				thisY = self.wayPointList[i][2]
				distance = np.sqrt((nextY-thisY)**2+(nextX-thisX)**2)
				T = distance/self.psudoVel
				self.wayPointList[i][0] = T
			else:
				self.wayPointList[i][0] = self.wayPointList[i][0]-self.wayPointList[i-1][0]

	def startingWaypointIndex(self, vehicleState):
		shortestDistance = float("inf")
		shortestIndex = 0;
		vehicleX = vehicleState[1]
		vehicleY = vehicleState[2]
		for i in range (0, len(self.wayPointList)):
			wayPointX = self.wayPointList[i][1] + self.x_offset
			wayPointY = self.wayPointList[i][2] + self.y_offset
			distance = (wayPointX-vehicleX)**2+(wayPointY-vehicleY)**2
			if distance <= shortestDistance:
				shortestDistance = distance
				shortestIndex = i
		print("starting at "+str(shortestIndex)+" waypoint")
		return shortestIndex

	def waypointToTraj(self, wayPointIndex, vehicleState):
		listLength = len(self.wayPointList)
		waypointSegmentList = np.zeros([self.trajNum+1, 5])
		waypointSegmentList[0] = vehicleState
		accTime = waypointSegmentList[0][0]
		for i in range(1,self.trajNum+1):
			tempState = self.wayPointList[(wayPointIndex+i)%listLength]
			accTime = accTime + tempState[0]
			tempState[0] = accTime
			tempState[1] = tempState[1]+self.x_offset
			tempState[2] = tempState[2]+self.y_offset
			waypointSegmentList[i] = tempState
		return generate_traj(waypointSegmentList)
