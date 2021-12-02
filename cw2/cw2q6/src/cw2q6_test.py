#!/usr/bin/env python
import numpy as np
import scipy
from scipy.linalg import expm
from scipy.linalg import logm
import rospy
import rosbag
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cw2q4.youbotKineKDL import YoubotKinematicKDL


import PyKDL
from visualization_msgs.msg import Marker
from itertools import permutations
# from sys import maxsize

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
# initial_pos = position[:,0]

# count = 0
distance = np.zeros((5,5))
# for i in range(position.shape[1]):
# if count == 0:

# current_pos = position[:,0]

for i in range(distance.shape[1]):
    for j in range(distance.shape[1]):
        distance[i,j] = np.linalg.norm(position[:,i]-position[:,j])
        # distance[:,i] = diff



# print(distance)
# print(target_cart_tf[:,:,0])
# print(target_cart_tf[:,:,1])
# print(target_cart_tf[:,:,2])
# print(target_cart_tf[:,:,3])
# print(target_cart_tf[:,:,4])

# print(position)
# print(initial_pos)


# implementation of traveling Salesman Problem

# store all vertex apart from source vertex

node = [] #create list for node
dist = [] # create a list for distance
# path = []
for i in range(distance.shape[1]):
	if i != 0:
		node.append(i) # this changes the list with [1 2 3 4]

# store minimum weight Hamiltonian Cycle
# min_path = maxsize
permutation=permutations(node) # this creates a permutation for the node list ie 1234 1243..
path = np.array(list(permutation)) # change the permutaion into a list and put it in an array
# path.append(list(permutation))

# path_perm = permutation
# path_perm = list(path_perm)
# path.append(path_perm)

for perm in path: # create loop for each permutation

	current_pathweight = 0 	# store current Path weight as 0 at initial pos
	# path.append(list(permutation))
	# compute current path weight
	k = 0
	for j in perm: # loop to add up all the distance of the permutation
		current_pathweight += distance[k][j]
		k = j
	dist.append(current_pathweight)
	# dist[:,i] = current_pathweight
	# path.append(list(permutation))

dis_val = np.zeros((24,1)) # convert distance list to an array although not necessary
for i in range(24): 
	dis_val[i,:] = dist[i]

min_distance = np.min(dis_val) #find minimum dis
min_index = [i for i, x in enumerate(dis_val) if x == min_distance]


min_path = path[min_index]
min_path = np.append(0, min_path)
# min_path = np.array(0,min_path)
# min_index = dist.index(min_distance)

assert isinstance(min_path, np.ndarray)
assert min_path.shape == (5,)
assert isinstance(min_distance, float)

# print(min_path.shape)

# path_and_dis = np.column_stack((path, dis_val))
# print(path_and_dis)
# print(dis_val)
# print(distance)
# print(path)
# print(dist)
# print(min_distance)

num_points = 3
checkpoint_a_tf = target_cart_tf[:, :, 0]
checkpoint_b_tf = target_cart_tf[:, :, 4]

t = 1/num_points
p_s = checkpoint_a_tf[0:3,3] 
p_f = checkpoint_b_tf[0:3,3] 

R_s = checkpoint_a_tf[0:3,0:3] 
R_f = checkpoint_b_tf[0:3,0:3] 
tfs = np.zeros((4,4,num_points))
# kyy= np.log(np.dot(np.linalg.inv(R_s),R_f))
# print(kyy)
for time in range(num_points):
	Pt = p_s + time*(p_f-p_s)
	Rt = R_s*scipy.linalg.expm(scipy.linalg.logm(np.dot(np.linalg.inv(R_s),R_f))*time)

	tf = np.column_stack((Rt, Pt))
	bot_row = np.array([0,0,0,1])
	tfs[:,:,time] = np.vstack((tf,bot_row))


# r = target_cart_tf[0:3,0:3,1]
# p = target_cart_tf[0:3,3,1]

# tf = np.column_stack((r, p))
# bot_row = np.array([0,0,0,1])
# tfs = np.vstack((tf,bot_row))

# print(r)
# print(p)
print(tfs[:,:,1])

