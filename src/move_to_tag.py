#! /usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import String
from math import pow, atan2, sqrt
from geometry_msgs.msg._Twist import Twist
from geometry_msgs.msg._Pose import Pose
from std_msgs.msg._Bool import Bool
from sensor_msgs.msg._LaserScan import LaserScan
import tf
from simple_camera.msg import Blob


class MoveToTag:

	def __init__(self):
		rospy.on_shutdown(self._shutdown)
		rospy.init_node('move_to_tag')

		#parameters checked
		self.blob_x = 0
		self.blob_y = 0
		self.blob_detected = False
		self.center_img = 620
		self.center_tolerance = 50
		self.start_driving = False
		self.driving_to_tag = False
		self.standing_on_tag = False

		self.stop = False
		self.pose = Pose()
		self.rate = rospy.Rate(20)
		self.obstacle = False

		# Subscriber

		self.pose_subscriber = rospy.Subscriber('/simple_odom_pose', Pose, self._update_pose)

		self.scanSub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)

		self.camera_subscriber = rospy.Subscriber('/blob', Blob, self._cameraBlob_callback)

		self.start_driving_subscriber = rospy.Subscriber('/move_to_tag_start_driving', Bool, self._move_to_tag)

		#TODO output
		#self.sub_goal_reached = rospy.Subscriber('/move_to_goal/reached', Bool, self._goal_reached_callback)

		# Publisher

		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		self.pub_goal = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)

		#TODO output
		#self.goal_reached_publisher = rospy.Publisher('move_to_tag/reached',  Bool, queue_size=1)
		
		print('--- ready ---')
		rospy.spin()

	def _shutdown(self):
		self.stop = True
		vel_msg = Twist()
		# Linear velocity in the x-axis.
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		# Angular velocity in the z-axis.
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		self.velocity_publisher.publish(vel_msg)
		self.rate.sleep()

	def _cameraBlob_callback(self,blob):
		self.blob_detected = blob.blob_detected
		self.blob_x = blob.blob_x
		self.blob_y = blob.blob_y

	def _scan_callback(self, scan):
		range_front = []
		min_front = 0
		# in front of the robot (between 20 to -20 degrees)
		range_front[:20] = scan.ranges[-20:]
		range_front[20:] = scan.ranges[:20]
		range_front = list(filter(lambda num: num != 0, range_front))
		min_front = min(range_front)
		if min_front < 0.18 and min_front != 0.0:
			self.obstacle = True
			print "Obstacle in Front"
		else:
			self.obstacle = False

	def _update_pose(self, data):
		"""
		Update current pose of robot
		"""
		try:
			self.pose.position.x = data.position.x
			self.pose.position.y = data.position.y
			self.pose.position.z = data.position.z
			self.pose.orientation.x = data.orientation.x 
			self.pose.orientation.y = data.orientation.y 
			self.pose.orientation.z = data.orientation.z
			self.pose.orientation.w = data.orientation.w 
		except:
			print('transform not ready')
		
		self.pose.position.x = round(data.position.x, 4)
		self.pose.position.y = round(data.position.y, 4)
		self.robot_yaw = self._robot_angle()
		

	def _robot_angle(self):
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(_, _, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
		return yaw

	def _move_to_tag(self, start_driving_bool):

		self.start_driving = start_driving_bool.data

		if self.start_driving == True and self.blob_detected == True:
			print('Drive to Tag')

			vel_msg = Twist()

			cancle = False

			while self.stop == False and cancle == False:

				#while not self.stop and not cancle:
				# Linear velocity in the x-axis.
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0

				if (self.blob_detected == True and self.blob_x < (self.center_img - self.center_tolerance)) or (self.blob_detected == True and self.blob_x > (self.center_img + self.center_tolerance)):
					print('rotate')
					vel_msg.angular.z = self._angular_vel()
				else:
					if not self.obstacle:
						if self.blob_detected == True:
							print('Forward')
							vel_msg.linear.x = self._linear_vel()
						else:
							print "no Blob in frame"
							self._calculate_last_point()
							cancle = True #1
					else:
						cancle = True
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0
						self.velocity_publisher.publish(vel_msg)

				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)

				# Publish at the desired rate.
				self.rate.sleep()

			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0

			self.velocity_publisher.publish(vel_msg)
			print "-------------------> Stop"

			self.rate.sleep()

	def _calculate_last_point(self):
		print "current x ->" + str(self.pose.position.x)
		print "current y ->" + str(self.pose.position.y)
		print "current yaw -->" + str(self.robot_yaw)

		next_x = self.pose.position.x + math.cos(self.robot_yaw) * 0.2
		next_y = self.pose.position.y + math.sin(self.robot_yaw) * 0.2

		print "next x ->" + str(next_x)
		print "next y ->" + str(next_y)
		print "next yaw -->" + str(self.robot_yaw)

		self._move_last_distance(next_x, next_y)

		print "TTTTTTTTTTTTTTTTTTTAAAAAAAAAAAAAAAAAAGGGGGGGGGGGGGGGGGGG"

	def _move_last_distance(self, x, y):
		print('Navigate to: ' + str(x) + ' | ' + str(y))
		goal = Pose()

		goal.position.x = x
		goal.position.y = y
		goal.orientation.w = 1

		self.pub_goal.publish(goal)

	def _linear_vel(self):
		return 0.03

	def _angular_vel(self):		
		
		if self.blob_x < ((self.center_img - self.center_tolerance) + 10):
			# rotate robot to the left
			return 0.075
		elif self.blob_x > ((self.center_img + self.center_tolerance) - 10):
			# rotate robot to the right
			return -0.075

if __name__ == '__main__':
	try:
		moveToTag=MoveToTag()
	except rospy.ROSInterruptException:
		pass
