#!/usr/bin/env python
import rospy, os
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from numpy import genfromtxt

global msg
global pub_path

global x_offset
global y_offset

x_offset = 0
y_offset = -3

# support functions
def pubWaypoints():
	global msg
	global pub_path
	global x_offset
	global y_offset
	# read the raw waypoints from the .csv file
	file_name = "ref_path_points.csv"
	target_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ),"..","data"))+"/"
	wayPointList = genfromtxt(target_path+file_name, delimiter=',')
	msg = Path()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "map"
	i = 0
	for waypoint in wayPointList:
		x = waypoint[1] + x_offset
		y = waypoint[2] + y_offset
		addToPath(x,y,i)
		i=i+1
	x = wayPointList[0][1] + x_offset
	y = wayPointList[0][2] + y_offset
	addToPath(x,y,i)
	pub_path.publish(msg)
	
def addToPath(x, y, seq):
	global msg
	global pub_path
	sub_msg = PoseStamped()
	sub_msg.header.stamp = rospy.Time.now()
	sub_msg.header.frame_id = "map"
	sub_msg.header.seq = seq
	sub_msg.pose.position.x = x
	sub_msg.pose.position.y = y
	sub_msg.pose.orientation.w = 1
	sub_msg.pose.orientation.x = 0
	sub_msg.pose.orientation.y = 0
	sub_msg.pose.orientation.z = 0
	msg.poses.append(sub_msg)

def main():
	global msg
	global pub_path
	rospy.init_node('refPath_node', anonymous=True)
	pub_path = rospy.Publisher('/vehicle/test/rviz/refPath2', Path, queue_size=1)
	while not rospy.is_shutdown():
		pubWaypoints()
		rospy.sleep(1)
	rospy.spin()


if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
