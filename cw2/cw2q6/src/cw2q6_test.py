#!/usr/bin/env python
import numpy as np
# from scipy.linalg import expm
# from scipy.linalg import logm
import rospy
import rosbag
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cw2q4.youbotKineKDL import YoubotKinematicKDL

import PyKDL
from visualization_msgs.msg import Marker
from itertools import permutations

rospack = rospkg.RosPack()
path = rospack.get_path('cw2q6')

# Initialize arrays for checkpoint transformations and joint positions
target_joint_positions = np.zeros((5, 5))
# Create a 4x4 transformation matrix, then stack 6 of these matrices together for each checkpoint
target_cart_tf = np.repeat(np.identity(4), 5, axis=1).reshape((4, 4, 5))

# Load path for selected question
bag = rosbag.Bag(path + '/bags/data.bag')
# Get the current starting position of the robot
target_joint_positions[:, 0] = YoubotKinematicKDL().kdl_jnt_array_to_list(YoubotKinematicKDL().current_joint_position)
# Initialize the first checkpoint as the current end effector position
target_cart_tf[:, :, 0] = YoubotKinematicKDL().forward_kinematics(target_joint_positions[:, 0])

# # Your code starts here ------------------------------
# joint_data = []
# for topic, msg, t in bag.read_messages(topics='joint_data'):
#     joint_data.append(msg.position)

# joint_targets = np.zeros((5, 4), dtype=float)

# # Given the hardcoded_joint_targets, load the joint targets into the numpy array variable joint_targets. Although
# # this is a somewhat unnecessary step, the template given in CW2 Q6 load_targets, loads the joint targets and
# # Cartesian checkpoints in this same way,
# # your code starts here ------------------------------
# for i in range(joint_targets.shape[1]):
#     joint_targets[:,i] = joint_data[i]

# # checkpoints = np.zeros([4, 4, joint_targets.shape[1]])
# #     for i in range(joint_targets.shape[1]):
# #         checkpoints[:,:,i] = kdl_youbot.forward_kinematics(list(joint_targets[: , i]))

joint_data = []
for topic, msg, t in bag.read_messages(topics='joint_data'):
    joint_data.append(msg.position)

for i in range(4):
    target_joint_positions[:,i+1] = joint_data[i]
    target_cart_tf[:,:,i+1] = YoubotKinematicKDL().forward_kinematics(target_joint_positions[:,i+1])

position = np.zeros((3,5))
for i in range(position.shape[1]):
    position[:,i] = target_cart_tf[0:3,3,i]

print(target_cart_tf[:,:,0])
print(target_cart_tf[:,:,1])
print(target_cart_tf[:,:,2])
print(target_cart_tf[:,:,3])
print(target_cart_tf[:,:,4])

print(position)


