#!/usr/bin/env python
from math import radians
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as msg
import sys
import copy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

# Initialising the command msg to be sent to the gripper
command = outputMsg.Robotiq2FGripper_robot_output()
command.rACT = 0
command.rGTO = 0
command.rATR = 0
command.rPR = 0  # only value you will change to open/close the gripper
command.rSP = 255
command.rFR = 150

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('kuka_move_test',
                anonymous=True)

robot = moveit_commander.RobotCommander()

rospy.loginfo('Gripper Init')
pub.publish(command)
rospy.sleep(1)

#scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)


rospy.loginfo("Waiting 2 seconds for intial setup...")
rospy.sleep(2)
rospy.loginfo("Ready to go!")

pick_approach = msg.Pose()
place_approach = msg.Pose()

# Brick position
brick = msg.Pose()
brick.position.x = 0.529277450769
brick.position.y = 0.442630265372
brick.position.z = 0.104465581135
brick.orientation.x = 0.778020560741
brick.orientation.y = 0.628106271376
brick.orientation.z = 0.00920160063765
brick.orientation.w = 0.00904855800551

# Pick approach
pick_approach.position.x = 0.519819576606
pick_approach.position.y = 0.433077872362
pick_approach.position.z = 0.33786534234
pick_approach.orientation.x = 0.764389574528
pick_approach.orientation.y = 0.644492649292
pick_approach.orientation.z = -0.0133650363344
pick_approach.orientation.w = 0.0126155234272

# Place approach
place_approach.position.x = 0.384539404522
place_approach.position.y = 0.326406689793
place_approach.position.z = 0.468987815643
place_approach.orientation.x = 0.778013586998
place_approach.orientation.y = 0.628115120395
place_approach.orientation.z = 0.00921072983402
place_approach.orientation.w = 0.00902258454604

# Front row 1
front = msg.Pose()
front.position.x = 0.613028226313
front.position.y = -0.0239848922303
front.position.z = 0.200020127018
front.orientation.x = 0.994473159313
front.orientation.y = -0.104197267266
front.orientation.z = 5.2166060977e-05
front.orientation.w = 0.0128863937736

# Back row 1
back = msg.Pose()
back.position.x = 0.602961760655
back.position.y = -0.14833045381
back.position.z = 0.190621861941
back.orientation.x = 0.994468927383
back.orientation.y = -0.104253626002
back.orientation.z = 0.000116945819938
back.orientation.w = 0.0127566036996

# Left row 1
left = msg.Pose()
left.position.x = 0.669045167266
left.position.y = -0.102593733982
left.position.z = 0.244247920598
left.orientation.x = 0.7780174613
left.orientation.y = 0.628109470738
left.orientation.z = 0.00914545443051
left.orientation.w = 0.00915003640455

# Right row 1
right = msg.Pose()
right.position.x = 0.548064812959
right.position.y = -0.0787853636518
right.position.z = 0.238078139926
right.orientation.x = 0.777987241745
right.orientation.y = 0.628151176539
right.orientation.z = 0.00899964819717
right.orientation.w = 0.00899713565298

# First brick
waypoints = []
waypoints.append(copy.deepcopy(pick_approach))
waypoints.append(copy.deepcopy(brick))

rospy.loginfo("Generating trajectory")

(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01, 
    0.0
)
rospy.loginfo("Executing")
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(15)

# Closing the gripper
command.rACT = 1
command.rGTO = 1
command.rPR = 225
pub.publish(command)
rospy.loginfo("Closing the gripper")
rospy.sleep(2)

# Elevate robot away block
waypoints = []
waypoints.append(copy.deepcopy(pick_approach))
(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0
)
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(5)

# Place the first block
waypoints = []
waypoints.append(copy.deepcopy(back))
(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0
)
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(15)

# Opening the gripper
command.rPR = 0
pub.publish(command) 
rospy.loginfo("Opening the gripper")
rospy.sleep(2)

# go to place approach
waypoints = []
waypoints.append(copy.deepcopy(place_approach))
(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0
)

rospy.loginfo("Executing")
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(10)

# Second brick
waypoints = []
waypoints.append(copy.deepcopy(pick_approach))
waypoints.append(copy.deepcopy(brick))

rospy.loginfo("Generating trajectory")

(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01, 
    0.0
)
rospy.loginfo("Executing")
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(15)

# Closing the gripper
command.rPR = 225
pub.publish(command)
rospy.loginfo("Closing the gripper")
rospy.sleep(2)

# Elevate robot away block
waypoints = []
waypoints.append(copy.deepcopy(pick_approach))
(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0
)
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(5)

# go to place approach
waypoints = []
waypoints.append(copy.deepcopy(place_approach))
(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0
)

rospy.loginfo("Executing")
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(5)

# Place block
waypoints = []
waypoints.append(copy.deepcopy(front))
(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0
)
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(15)

# Opening the gripper
command.rPR = 0
pub.publish(command) 
rospy.loginfo("Opening the gripper")
rospy.sleep(2)

# go to place approach
waypoints = []
waypoints.append(copy.deepcopy(place_approach))
(plan, friction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0
)

rospy.loginfo("Executing")
group.execute(plan, wait=True)
group.clear_pose_targets()
rospy.sleep(10)

# # Third brick
# waypoints = []
# waypoints.append(copy.deepcopy(pick_approach))
# waypoints.append(copy.deepcopy(brick))

# rospy.loginfo("Generating trajectory")

# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01, 
#     0.0
# )
# rospy.loginfo("Executing")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(15)

# # Closing the gripper
# command.rPR = 225
# pub.publish(command)
# rospy.loginfo("Closing the gripper")
# rospy.sleep(2)

# # Elevate robot away block
# waypoints = []
# waypoints.append(copy.deepcopy(pick_approach))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(5)

# # go to place approach
# waypoints = []
# waypoints.append(copy.deepcopy(place_approach))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )

# rospy.loginfo("Executing")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(5)

# # Place block
# waypoints = []
# waypoints.append(copy.deepcopy(left))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(15)

# # Opening the gripper
# command.rPR = 0
# pub.publish(command) 
# rospy.loginfo("Opening the gripper")
# rospy.sleep(2)

# # go to place approach
# waypoints = []
# waypoints.append(copy.deepcopy(place_approach))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )

# rospy.loginfo("Executing")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(10)

# # Fourth brick
# waypoints = []
# waypoints.append(copy.deepcopy(pick_approach))
# waypoints.append(copy.deepcopy(brick))

# rospy.loginfo("Generating trajectory")

# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01, 
#     0.0
# )
# rospy.loginfo("Executing")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(15)

# # Closing the gripper
# command.rPR = 225
# pub.publish(command)
# rospy.loginfo("Closing the gripper")
# rospy.sleep(2)

# # Elevate robot away block
# waypoints = []
# waypoints.append(copy.deepcopy(pick_approach))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(5)

# # go to place approach
# waypoints = []
# waypoints.append(copy.deepcopy(place_approach))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )

# rospy.loginfo("Executing")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(5)

# # Place block
# waypoints = []
# waypoints.append(copy.deepcopy(right))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(15)

# # Opening the gripper
# command.rPR = 0
# pub.publish(command) 
# rospy.loginfo("Opening the gripper")
# rospy.sleep(2)

# # go to place approach
# waypoints = []
# waypoints.append(copy.deepcopy(place_approach))
# (plan, friction) = group.compute_cartesian_path(
#     waypoints,
#     0.01,
#     0.0
# )

# rospy.loginfo("Executing")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(10)