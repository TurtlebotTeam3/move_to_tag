#! /usr/bin/env python

import rospy
import numpy as np
import math
import tf
import time

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
		rospy.init_node('move_to_tag')
		rospy.loginfo('Move to Tag node started')
		rospy.on_shutdown(self._shutdown)

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

		rospy.loginfo("--- publisher ---")
        # --- Publishers ---
		self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.move_to_goal_publisher = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)
		self.stop_move_to_goal_publisher = rospy.Publisher('move_to_goal/pause_action', Bool, queue_size=1)
		self.move_to_tag_reached_publisher = rospy.Publisher('move_to_tag/reached', Bool, queue_size=1)

		rospy.loginfo("--- subscriber ---")
        # --- Subscribers ---
		self.pose_subscriber = rospy.Subscriber('simple_odom_pose', CustomPose, self._handle_update_pose)
		self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self._scan_callback)
		self.camera_subscriber = rospy.Subscriber('blob', Blob, self._camera_blob_callback)
		self.start_driving_subscriber = rospy.Subscriber('move_to_tag_start_driving', Bool, self._move_to_tag)
		self.goal_reached_subscriber = rospy.Subscriber('move_to_goal/reached', Bool, self._goal_reached_callback)

		rospy.loginfo("--- service wait ---")
        # --- Service wait ---
		rospy.loginfo("1")
		rospy.wait_for_service('add_tag')

		rospy.loginfo("--- services ---")
		self.tag_manager_add_service = rospy.ServiceProxy('add_tag', AddTag)

		self.last_pose_time = None

		self._setup()
		rospy.loginfo('--- ready ---')
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
		"""
		Get map meta information.
		"""
		map = rospy.wait_for_message('map', OccupancyGrid)
		self.map_info = map.info

	def _goal_reached_callback(self, reached):
		"""
		Reached information from move_to_goal.
		"""
		# if robot is driving to tag then check if goal was reached.
		if self.driving_to_tag == True:
			if reached:
				self.driving_to_tag = False
				self.standing_on_tag = True
				add_service_response = self.tag_manager_add_service(self.pose_converted.x,self.pose_converted.y)
				self.move_to_tag_reached_publisher.publish(True)
				rospy.loginfo("--> reached tag and added")
			else:
				self.driving_to_tag = False
				self.standing_on_tag = False
				self.stop_move_to_goal_publisher.publish(False)
				rospy.loginfo("ERROR --> MOVE GOAL ERROR")

	def _camera_blob_callback(self,blob):
		"""
		Position of detected blob in the frame.
		"""
		self.blob_detected = blob.blob_detected
		self.blob_x = blob.blob_x
		self.blob_y = blob.blob_y
		self._check_if_blob_lost_bottom(self.blob_x,self.blob_y)

	def _check_if_blob_lost_bottom(self, x, y):
		"""
		Function to check if the blob is lost
		in the lower half of the picture.
		"""
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
		"""
		Lidar data, check for obstacles.
		"""
		range_front = []
		min_front = 0
		# in front of the robot (between 20 to -20 degrees)
		range_front[:20] = scan.ranges[-20:]
		range_front[20:] = scan.ranges[:20]
		range_front = list(filter(lambda num: num != 0, range_front))
		min_front = min(range_front)
		if min_front < 0.25 and min_front != 0.0:
			self.stop_move_to_goal_publisher.publish(False)
			self.obstacle = True
		else:
			self.obstacle = False

	def _handle_update_pose(self, data):
		"""
		Update current pose of robot
		"""
		try:
			self.last_pose_time = int(round(time.time() * 1000))

			self.pose = data.pose
			self.pose_converted = data.pose_converted
		except:
			rospy.loginfo("ERROR --> TRANSFORM NOT READY")

	def _move_to_tag(self, start_driving_bool):
		"""
		Center blob in the middle. Then drive straight ahead.
		If the blob is lost in the lower half of the image, 
		drive the last distance blind.
		"""
		if start_driving_bool.data == True and self.blob_detected == True:
			self.start_driving = start_driving_bool.data
			rospy.loginfo('--> drive to tag')
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

				
				if not self.obstacle:
						current_time_ms = int(round(time.time() * 1000))
						# Check if the network is working properly
						if self.last_pose_time != None and (current_time_ms - self.last_pose_time) < 50:
							# center the blob if not in center
							if (self.blob_detected == True and self.blob_x < (self.center_img - self.center_tolerance)) or \
									(self.blob_detected == True and self.blob_x > (self.center_img + self.center_tolerance)):
									vel_msg.angular.z = self._angular_vel()
									vel_msg.linear.x = 0.015
							# move straight
							elif self.blob_detected == True:
								vel_msg.linear.x = self._linear_vel()
							else:
								if self.blob_lost_at_bottom == True:
									rospy.loginfo("--> blob lost at bottom dive to last point")
									cancle = True
									# Blob disapeard at the bottom of the frame, no calculate position so robot stops on tag.
									self._calculate_last_point()
								else: 
									rospy.loginfo("ERROR --> BLOB LOST CANCLE MOVE TO TAG")
									cancle = True
									self.driving_to_tag = False
									self.standing_on_tag = False
									self.stop_move_to_goal_publisher.publish(False)
						else:
							vel_msg.linear.x = 0
							vel_msg.angular.z = 0
							self.velocity_publisher.publish(vel_msg)
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
		"""
		Calculate how far robot needs to drive to be positioned on tag.
		"""
		next_x = round(self.pose.position.x, 4) + math.cos(self.pose_converted.yaw) * 0.175
		next_y = round(self.pose.position.y, 4) + math.sin(self.pose_converted.yaw) * 0.175

		self._move_last_distance(next_x, next_y)

	def _move_last_distance(self, x, y):
		"""
		Drive the last distance to stand on the marker 
		"""
		rospy.loginfo('--> navigate to: ' + str(x) + ' | ' + str(y))
		goal = Pose()

		goal.position.x = x
		goal.position.y = y
		goal.orientation.w = 1

		self.move_to_goal_publisher.publish(goal)
		self.stop_move_to_goal_publisher.publish(False)

	def _linear_vel(self):
		"""
		Set linar velocity
		"""
		return 0.075

	def _angular_vel(self):
		"""
		Set angular velocity
		"""	
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
