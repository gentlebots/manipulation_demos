#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages
#		
#		* Jonatan Gines

import rospy
import time
from gb_tiago_manipulation_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty

import cv2
from cv_bridge import CvBridge

from moveit_msgs.msg import MoveItErrorCodes, Grasp, PlaceLocation
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name

class SphericalService(object):
	def __init__(self):
		rospy.loginfo("Starting Spherical Grab Service")
		self.pick_type = PickObject()
		rospy.loginfo("Finished SphericalService constructor")
		self.place_gui = rospy.Service("/place_gui", Empty, self.start_object_place)
		self.pick_sub = rospy.Subscriber("/moveit/pick", Grasp, self.pick_cb)
		self.place_sub = rospy.Subscriber("/moveit/place", PlaceLocation, self.place_cb)

	def start_object_place(self, req):
		self.pick_type.pick("place")
		return {}
	
	def pick_cb(self, msg):
		self.pick_type.pick(msg.id, msg.grasp_pose)
		return {}
	
	def place_cb(self, msg):
		self.pick_type.place(msg.id, msg.place_pose)
		return {}

	def step(self):
		return {}

class PickObject(object):
	def __init__(self):
		rospy.loginfo("Initalizing...")
		self.tfBuffer = tf2_ros.Buffer()
		self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
		rospy.loginfo("Waiting for /pickup_pose AS...")
		self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
		time.sleep(2.0)
		if not self.pick_as.wait_for_server(rospy.Duration(15)):
			rospy.logerr("Could not connect to /pickup_pose AS")
		rospy.loginfo("Waiting for /place_pose AS...")
		self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)

		self.place_as.wait_for_server()

		rospy.loginfo("Setting publishers to torso and head controller...")
		self.torso_cmd = rospy.Publisher(
			'/torso_controller/command', JointTrajectory, queue_size=1)
		self.head_cmd = rospy.Publisher(
			'/head_controller/command', JointTrajectory, queue_size=1)
		self.detected_pose_pub = rospy.Publisher('/detected_object_pose',
			PoseStamped,
			queue_size=1,
			latch=True)
		self.result_pub = rospy.Publisher('/moveit/result', MoveItErrorCodes, queue_size=1)

		rospy.loginfo("Waiting for '/play_motion' AS...")
		self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
		if not self.play_m_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /play_motion AS")
			exit()
		rospy.loginfo("Connected!")
		rospy.sleep(0.5)
		rospy.loginfo("Done initializing Pickobject.")

   	def strip_leading_slash(self, s):
		return s[1:] if s.startswith("/") else s

	def pick(self, object_id, object_pose):
		self.prepare_robot()
		rospy.loginfo("Got: " + str(object_pose))
		rospy.sleep(3.0)
		rospy.loginfo("spherical_grasp_gui: Transforming from frame: " +
		object_pose.header.frame_id + " to 'base_footprint'")
		ps = PoseStamped()
		ps.pose = object_pose.pose
		ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", object_pose.header.frame_id)
		ps.header.frame_id = object_pose.header.frame_id
		transform_ok = False
		while not transform_ok and not rospy.is_shutdown():
			try:
				transform = self.tfBuffer.lookup_transform("base_footprint", 
									   ps.header.frame_id,
									   rospy.Time(0))
				object_ps = do_transform_pose(ps, transform)
				transform_ok = True
			except tf2_ros.ExtrapolationException as e:
				rospy.logwarn(
					"Exception on transforming point... trying again \n(" +
					str(e) + ")")
				rospy.sleep(0.01)
				ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", object_pose.header.frame_id)
			pick_g = PickUpPoseGoal()
			rospy.loginfo("Setting cube pose based on object detection")
			pick_g.object_pose.pose = object_ps.pose
			pick_g.object_pose.header.frame_id = object_pose.header.frame_id
			pick_g.object_id = object_id
			rospy.loginfo("object pose in base_footprint:" + str(pick_g))
			
			self.detected_pose_pub.publish(pick_g.object_pose)
			rospy.loginfo("Gonna pick:" + str(pick_g))
			self.pick_as.send_goal_and_wait(pick_g)
			rospy.loginfo("Done!")

			result = self.pick_as.get_result()
			
			if str(moveit_error_dict[result.error_code]) != "SUCCESS":
				rospy.logerr("Failed to pick, not trying further")

			# Move torso to its maximum height
			self.lift_torso()
			# Raise arm

			rospy.loginfo("Moving arm to a safe pose")
			self.arm_to_home()
			rospy.loginfo("Raise object done.")
			result_msg = MoveItErrorCodes()
			result_msg.val = result.error_code
			self.result_pub.publish(result_msg)

	def place(self, place_id, pose):
		self.lift_torso()
		self.lower_head()
		rospy.loginfo("Got: " + str(pose))
		rospy.sleep(3.0)
		rospy.loginfo("Transforming from frame: " + pose.header.frame_id + " to 'base_footprint'")
		ps = PoseStamped()
		ps.pose = pose.pose
		ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", pose.header.frame_id)
		ps.header.frame_id = pose.header.frame_id
		transform_ok = False
		while not transform_ok and not rospy.is_shutdown():
			try:
				transform = self.tfBuffer.lookup_transform("base_footprint", 
									   ps.header.frame_id,
									   rospy.Time(0))
				object_ps = do_transform_pose(ps, transform)
				transform_ok = True
			except tf2_ros.ExtrapolationException as e:
				rospy.logwarn(
					"Exception on transforming point... trying again \n(" +
					str(e) + ")")
				rospy.sleep(0.01)
				ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", pose.header.frame_id)
			place_g = PickUpPoseGoal()
			#rospy.loginfo("Setting cube pose based on object detection")
			place_g.object_pose.pose = object_ps.pose
			place_g.object_pose.header.frame_id = "base_footprint"
			rospy.loginfo("Place pose in base_footprint:" + str(place_g))
			#self.detected_pose_pub.publish(object_ps.pose)

			# Place the object back to its position
			place_g.object_pose.pose.position.z += 0.05
			self.place_as.send_goal_and_wait(place_g)
			rospy.loginfo("Done!")
			result = self.pick_as.get_result()
			if str(moveit_error_dict[result.error_code]) != "SUCCESS":
				rospy.logerr("Failed to pick, not trying further")
				return

			self.arm_to_home()
			result_msg = MoveItErrorCodes()
			result_msg.val = result.error_code
			self.result_pub.publish(result_msg)
	
	def arm_to_home(self):
		pmg = PlayMotionGoal()
		pmg.motion_name = 'home'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)
	
	def lift_torso(self):
		rospy.loginfo("Moving torso up")
		jt = JointTrajectory()
		jt.joint_names = ['torso_lift_joint']
		jtp = JointTrajectoryPoint()
		jtp.positions = [0.34]
		jtp.time_from_start = rospy.Duration(2.5)
		jt.points.append(jtp)
		self.torso_cmd.publish(jt)
	
	def lower_head(self):
		rospy.loginfo("Moving head down")
		jt = JointTrajectory()
		jt.joint_names = ['head_1_joint', 'head_2_joint']
		jtp = JointTrajectoryPoint()
		jtp.positions = [0.0, -0.90]
		jtp.time_from_start = rospy.Duration(2.0)
		jt.points.append(jtp)
		self.head_cmd.publish(jt)
		rospy.loginfo("Done.")

	def prepare_robot(self):
		#rospy.loginfo("Unfold arm safely")
		#pmg = PlayMotionGoal()
		#pmg.motion_name = 'pregrasp'
		#pmg.skip_planning = False
		#self.play_m_as.send_goal_and_wait(pmg)
		rospy.loginfo("Done.")
		self.lower_head()
		rospy.loginfo("Robot prepared.")

if __name__ == '__main__':
	rospy.init_node('pick_demo')
	sphere = SphericalService()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		sphere.step()
		rate.sleep()
