#!/usr/bin/env python
import numpy as np

def generate_traj(wayPointList):
	'''
	INPUT: the wayPoint List is a N*5 matrix:
	[[t0, x0, y0, xdot0, ydot0]
	 [t1, x1, y1, xdot1, ydot1]
	 [t2, x2, y2, xdot2, ydot2]
	 ...   ...  ...  ...  ...  ]
	Note that N represents the number of the given waypoints, 
	and each 5-D vector represents vehicle's instant state, 
	shown as the form [t, xPos, vPos, xVel, yvel].

	OUTPUT: The function produces a (N-1)*13 of matrix, which means
	it generates N-1 trajectories. Each 13-D vector was formed by one
	time interval, and two 6-polynomial coefficient sets in both x and 
	y axis.
	'''
	if np.size(wayPointList) < 10:
		raise IndexError('There must be at least two way points.')
		return [];
	else:
		trajsNum = len(wayPointList)-1
		vecList = np.zeros([trajsNum,13])

		for i in range (0,trajsNum):
			T = wayPointList[i+1][0] - wayPointList[i][0]
			x0 = wayPointList[i][1]
			y0 = wayPointList[i][2]
			dx0 = wayPointList[i][3]
			dy0 = wayPointList[i][4]
			
			xf = wayPointList[i+1][1]
			yf = wayPointList[i+1][2]
			dxf = wayPointList[i+1][3]
			dyf = wayPointList[i+1][4]
			# assime that vehicle's instant acceleration is zero.
			vecList[i][0] = T
			vecList[i][1:7] = minimumJerkTraj(x0, dx0, 0, xf, dxf, 0, T)
			vecList[i][7:13] = minimumJerkTraj(y0, dy0, 0, yf, dyf, 0, T)
		return vecList


			


def minimumJerkTraj(x0, dx0, ddx0, xT, dxT, ddxT, T):
	'''
	Compute a point to point minimum jerk trajectory
	x0 dx0 ddx0 are the location, velocity and acceleration at the
	start point
	xT dxT ddxT are the target location velocity and acceleration
	T is the time required to move from the start point
	to the target point
	
	The solution is a 6-D vector of coefficients a
	The minimum jerk trajectory takes the form
	x_t = \sum_{k=1}^6 a_k t^(k-1), for 0\leq t \leq T
	
	Copyright Javier R. Movellan UCSD 2011
	The code was transcripted from MATLAB to Python by Jinwei Zzhang @ 2018
	'''
	T2=T**2
	T3=T**3
	T4=T**4
	T5=T**5
	a = np.zeros([6,1])
	a[0, 0] = x0
	a[1, 0] = dx0
	a[2, 0] = ddx0/2
	b = np.array([[T3, T4, T5], [3*T2, 4*T3, 5*T4], [6*T, 12*T2, 20*T3]])
	c = np.array([[xT-a[0 ,0]-a[1 ,0]*T-a[2 ,0]*T2], [dxT-a[1 ,0]-2*a[2 ,0]*T], [ddxT-2*a[2 ,0]]])
	a[3:6] = np.dot(np.linalg.pinv(b),c)
	return np.transpose(a)[0];

if __name__ == '__main__':
	# print(minimumJerkTraj(0,0,0,5,0,0,5))
	wayPointList = [[0,0,0,0,0],[5,5,5,0,0],[10,20,0,0,0]]
	generate_traj(wayPointList)
