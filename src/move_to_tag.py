#! /usr/bin/env python

import rospy
import numpy as np
import math
import tf

from std_msgs.msg import String, Bool
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData

from simple_camera.msg import Blob
from tag_manager.srv import AddTag
from simple_odom.msg import PoseConverted, CustomPose

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
		self.blob_lost_at_bottom = False
		self.stop = False
		
		self.rate = rospy.Rate(20)
		self.obstacle = False
		self.last_x = 0
		self.last_y = 0

		self.map_info = MapMetaData()

		self.pose = Pose()
		self.pose_converted = PoseConverted()

		# Subscriber
		self.pose_subscriber = rospy.Subscriber('simple_odom_pose', CustomPose, self._handle_update_pose)

		self.scanSub = rospy.Subscriber('scan', LaserScan, self._scan_callback)

		self.camera_subscriber = rospy.Subscriber('blob', Blob, self._cameraBlob_callback)

		self.start_driving_subscriber = rospy.Subscriber('move_to_tag_start_driving', Bool, self._move_to_tag)

		self.sub_goal_reached = rospy.Subscriber('move_to_goal/reached', Bool, self._goal_reached_callback)

		# Publisher

		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		self.pub_goal = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)

		self.move_to_goal_cansel = rospy.Publisher('move_to_goal/cancel', Bool, queue_size=1)

		#stop move to goal
		self.stop_move_to_goal_publisher = rospy.Publisher('move_to_goal/pause_action', Bool, queue_size=1)

		rospy.wait_for_service('add_tag')
		print "--- wait for add_tag service"
		self.tag_manager_add = rospy.ServiceProxy('add_tag', AddTag)
		print "--- add_tag service ready"
		
		self._setup()

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

	def _setup(self):
		map = rospy.wait_for_message('map', OccupancyGrid)
		self.map_info = map.info


	def _goal_reached_callback(self, reached):
		if self.driving_to_tag == True:
			if reached:
				self.driving_to_tag = False
				self.standing_on_tag = True
				#save tag in tag manager
				add_service_response = self.tag_manager_add(self.pose_converted.x,self.pose_converted.y)
				print "--> reached tag and added"
				print str(add_service_response)
				#TODO stop blob detection
			else:
				self.driving_to_tag = False
				self.standing_on_tag = False
				self.stop_move_to_goal_publisher.publish(False)
				print "ERROR --> MOVE GOAL ERROR"

	def _cameraBlob_callback(self,blob):
		self.blob_detected = blob.blob_detected
		self.blob_x = blob.blob_x
		self.blob_y = blob.blob_y
		self._check_if_blob_lost_bottom(self.blob_x,self.blob_y)

	def _check_if_blob_lost_bottom(self, x, y):
		if x != 0:
			self.last_x = x
		if y != 0:
			self.last_y = y
		if self.blob_detected == False:
			if self.last_y > 550:
				self.blob_lost_at_bottom = True
			else: 
				self.blob_lost_at_bottom = False

	def _scan_callback(self, scan):
		range_front = []
		min_front = 0
		# in front of the robot (between 20 to -20 degrees)
		range_front[:20] = scan.ranges[-20:]
		range_front[20:] = scan.ranges[:20]
		range_front = list(filter(lambda num: num != 0, range_front))
		min_front = min(range_front)
		if min_front < 0.18 and min_front != 0.0:
			self.stop_move_to_goal_publisher.publish(False)
			self.obstacle = True
		else:
			self.obstacle = False

	def _handle_update_pose(self, data):
		"""
		Update current pose of robot
		"""
		try:
			self.pose = data.pose
			self.pose_converted = data.pose_converted
		except:
			print "ERROR --> TRANSFORM NOT READY"

	def _move_to_tag(self, start_driving_bool):
		if start_driving_bool.data == True and self.blob_detected == True:
			self.start_driving = start_driving_bool.data
			print('--> drive to tag')

			vel_msg = Twist()

			cancle = False

			while self.stop == False and cancle == False and self.obstacle == False:
				self.driving_to_tag = True
				self.standing_on_tag = False

				#while not self.stop and not cancle:
				# Linear velocity in the x-axis.
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0

				if (self.blob_detected == True and self.blob_x < (self.center_img - self.center_tolerance)) or \
					(self.blob_detected == True and self.blob_x > (self.center_img + self.center_tolerance)):
					print('--> rotate')
					vel_msg.angular.z = self._angular_vel()
					vel_msg.linear.x = 0.015
				else:
					if not self.obstacle:
						if self.blob_detected == True:
							print('--> forward')
							vel_msg.linear.x = self._linear_vel()
						else:
							if self.blob_lost_at_bottom == True:
								print "--> blob lost at bottom dive to last point"
								cancle = True
								self._calculate_last_point()
							else: 
								print "ERROR --> BLOB LOST CANCLE MOVE TO TAG"
								cancle = True
								self.driving_to_tag = False
								self.standing_on_tag = False
								self.stop_move_to_goal_publisher.publish(False)
					else:
						cancle = True
						self.driving_to_tag = False
						self.standing_on_tag = False
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0
						self.velocity_publisher.publish(vel_msg)
						self.stop_move_to_goal_publisher.publish(False)

				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)

				# Publish at the desired rate.
				self.rate.sleep()

			print "--> end of move to tag"
			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0

			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
			self.stop_move_to_goal_publisher.publish(False)
		
		self.stop_move_to_goal_publisher.publish(False)

	def _calculate_last_point(self):
		next_x = round(self.pose.position.x, 4) + math.cos(self.pose_converted.yaw) * 0.175
		next_y = round(self.pose.position.y, 4) + math.sin(self.pose_converted.yaw) * 0.175

		self._move_last_distance(next_x, next_y)

	def _move_last_distance(self, x, y):
		print('--> navigate to: ' + str(x) + ' | ' + str(y))
		goal = Pose()

		goal.position.x = x
		goal.position.y = y
		goal.orientation.w = 1

		self.pub_goal.publish(goal)
		self.stop_move_to_goal_publisher.publish(False)

	def _linear_vel(self):
		return 0.075

	def _angular_vel(self):	
		if self.blob_x < ((self.center_img - self.center_tolerance) + 10):
			# rotate robot to the left
			return 0.1
		elif self.blob_x > ((self.center_img + self.center_tolerance) - 10):
			# rotate robot to the right
			return -0.1

if __name__ == '__main__':
	try:
		moveToTag=MoveToTag()
	except rospy.ROSInterruptException:
		pass
