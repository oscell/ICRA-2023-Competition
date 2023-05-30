#!/usr/bin/env python
from math import radians
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as msg
import sys
import copy

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

#scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)


rospy.loginfo("Waiting 2 seconds for intial setup...")
rospy.sleep(2)
rospy.loginfo("Ready to go!")

pick_approach = msg.Pose()

pick_approach.position.x = -0.411734572337
pick_approach.position.y = -0.5582854026
pick_approach.position.z = 0.354717573002
pick_approach.orientation.x = 0.681604271281
pick_approach.orientation.y = 0.731102429459
pick_approach.orientation.z = -0.000695229454232
pick_approach.orientation.w = 0.0300727728485

brick3_pick = msg.Pose()
brick3_pick.position.x = -0.411759555573
brick3_pick.position.y = -0.558342844345
brick3_pick.position.z = 0.179747069264
brick3_pick.orientation.x = 0.681534482621
brick3_pick.orientation.y = 0.731170695292
brick3_pick.orientation.z = -0.000868861873143
brick3_pick.orientation.w = 0.0299901387745

waypoints = []

start_pose = group.get_current_pose().pose
rospy.loginfo(start_pose)
# waypoints.append(copy.deepcopy(start_pose))
waypoints.append(copy.deepcopy(pick_approach))
waypoints.append(copy.deepcopy(start_pose))


rospy.loginfo("Generating trajectory")

(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01, 
    0.0
)

rospy.loginfo("Executing")
group.execute(plan, wait=True)