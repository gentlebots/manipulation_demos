#! /usr/bin/env python
# -*- coding: utf-8 -*-

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

import rospy
import roslib
from spherical_grasps_server import SphericalGrasps
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation, JointConstraint
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from gb_tiago_manipulation_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from random import shuffle
import dynamic_reconfigure.client
import copy

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[],
					 links_to_allow_contact=None):
	""" Create a PickupGoal with the provided data"""
	pug = PickupGoal()
	pug.target_name = target
	pug.group_name = group
	pug.possible_grasps.extend(possible_grasps)
	pug.allowed_planning_time = 20.0

	#j = JointConstraint()
	#j.joint_name = "arm_7_joint"
	#j.position = 1.0821
	#j.tolerance_above = 0.1
	#j.tolerance_below = 0.1
	#j.weight = 1.0
	#pug.path_constraints.joint_constraints = [j]

	pug.planning_options.planning_scene_diff.is_diff = True
	pug.planning_options.planning_scene_diff.robot_state.is_diff = True
	pug.planning_options.plan_only = False
	pug.planning_options.replan = True
	pug.planning_options.replan_attempts = 10  # 10
	pug.allowed_touch_objects = []
	pug.attached_object_touch_links = ['<octomap>']
	pug.attached_object_touch_links.extend(links_to_allow_contact)
	return pug


def createPlaceGoal(place_pose,
					place_locations,
					group="arm_torso",
					target="part",
					links_to_allow_contact=None):
	"""Create PlaceGoal with the provided data"""
	placeg = PlaceGoal()
	placeg.group_name = group
	placeg.attached_object_name = target
	placeg.place_locations = place_locations
	placeg.allowed_planning_time = 10.0
	placeg.planning_options.planning_scene_diff.is_diff = True
	placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
	placeg.planning_options.plan_only = False
	placeg.planning_options.replan = True
	placeg.planning_options.replan_attempts = 20
	placeg.allowed_touch_objects = ['<octomap>']
	placeg.allowed_touch_objects.extend(links_to_allow_contact)

	return placeg

class PickAndPlaceServer(object):
	def __init__(self):
		rospy.loginfo("Initalizing PickAndPlaceServer...")
		self.sg = SphericalGrasps()
		rospy.loginfo("Connecting to pickup AS")
		self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
		self.pickup_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		rospy.loginfo("Connecting to place AS")
		self.place_ac = SimpleActionClient('/place', PlaceAction)
		self.place_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		self.scene = PlanningSceneInterface()
		rospy.loginfo("Connecting to /get_planning_scene service")
		self.scene_srv = rospy.ServiceProxy(
			'/get_planning_scene', GetPlanningScene)
		self.scene_srv.wait_for_service()
		rospy.loginfo("Connected.")

		self.dyn_client = dynamic_reconfigure.client.Client('pick_and_place_server', timeout=30)

		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy(
			'/clear_octomap', Empty)
		self.clear_octomap_srv.wait_for_service()
		rospy.loginfo("Connected!")
		# Get the links of the end effector exclude from collisions
		self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)
		if self.links_to_allow_contact is None:
			rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
		else:
			rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))

		self.pick_as = SimpleActionServer(
			'/pickup_pose', PickUpPoseAction,
			execute_cb=self.pick_cb, auto_start=False)
		self.pick_as.start()

		self.place_as = SimpleActionServer(
			'/place_pose', PickUpPoseAction,
			execute_cb=self.place_cb, auto_start=False)
		self.place_as.start()

	def pick_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.grasp_object(goal.object_pose, goal.object_id)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.pick_as.set_aborted(p_res)
		else:
			self.pick_as.set_succeeded(p_res)

	def place_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.place_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.place_as.set_aborted(p_res)
		else:
			self.place_as.set_succeeded(p_res)

	def wait_for_planning_scene_object(self, object_name='part'):
		rospy.loginfo(
			"Waiting for object '" + object_name + "'' to appear in planning scene...")
		gps_req = GetPlanningSceneRequest()
		gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
		
		part_in_scene = False
		while not rospy.is_shutdown() and not part_in_scene:
			# This call takes a while when rgbd sensor is set
			gps_resp = self.scene_srv.call(gps_req)
			# check if 'part' is in the answer
			for collision_obj in gps_resp.scene.world.collision_objects:
				if collision_obj.id == object_name:
					part_in_scene = True
					break
			else:
				rospy.sleep(1.0)

		rospy.loginfo("'" + object_name + "'' is in scene!")

	def grasp_object(self, object_pose, object_id):
		rospy.loginfo("Removing any previous 'part' object")
		self.scene.remove_attached_object("arm_tool_link")
		self.scene.remove_world_object("part")
		rospy.loginfo("Clearing octomap. Calling srv...")
		self.clear_octomap_srv.call(EmptyRequest())
		rospy.sleep(0.5)  # Removing is fast
		rospy.loginfo("Adding new 'part' object")
		rospy.loginfo("Object %s: %s", object_id, object_pose)
		
		# Get the object dae path and size from a yaml.
		self.obj_properties = rospy.get_param('/object_properties/' + object_id, None)
		if self.obj_properties == None:
			rospy.logerr("Error loading object properties for %s", object_id)
			return 99999
		path = roslib.packages.get_pkg_dir('tmc_wrs_gazebo_worlds')+self.obj_properties['dae']
		params = {}
		self.scene.add_mesh("part", object_pose, path)
		for prop in self.obj_properties:
			if prop != 'dae': 
				params[prop] = self.obj_properties[prop]

		rospy.loginfo("Updating SphericalGrasps params...")
		self.dyn_client.update_configuration(params)
		
		grasp_pose = copy.deepcopy(object_pose)
		grasp_pose.pose.position.z += 0.04

		rospy.loginfo("Grasp pose --- %s", grasp_pose.pose)
		possible_grasps = self.sg.create_grasps_from_object_pose(grasp_pose)
		self.pickup_ac
		goal = createPickupGoal(
			"arm_torso", "part", grasp_pose, possible_grasps, self.links_to_allow_contact)
		rospy.loginfo("Sending goal")
		self.pickup_ac.send_goal(goal)
		rospy.loginfo("Waiting for result")
		self.pickup_ac.wait_for_result()
		result = self.pickup_ac.get_result()
		rospy.logdebug("Using torso result: " + str(result))
		rospy.loginfo(
			"Pick result: " +
		str(moveit_error_dict[result.error_code.val]))
		return result.error_code.val

	def place_object(self, object_pose):
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())
		rospy.loginfo("Trying to place with arm and torso")
		possible_placings = self.sg.create_placings_from_object_pose(
			object_pose)
		# Try with arm and torso
		goal = createPlaceGoal(
				object_pose, possible_placings, "arm_torso", "part", self.links_to_allow_contact)
		rospy.loginfo("Sending goal")
		self.place_ac.send_goal(goal)
		rospy.loginfo("Waiting for result")

		self.place_ac.wait_for_result()
		result = self.place_ac.get_result()
		rospy.logerr(str(moveit_error_dict[result.error_code.val]))

		rospy.loginfo(
			"Result: " +
			str(moveit_error_dict[result.error_code.val]))
		rospy.loginfo("Removing previous 'part' object")
		self.scene.remove_world_object("part")

		return result.error_code.val


if __name__ == '__main__':
	rospy.init_node('pick_and_place_server')
	paps = PickAndPlaceServer()
	rospy.spin()
