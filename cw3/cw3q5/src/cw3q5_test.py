#!/usr/bin/env python

import numpy as np
import rospy
import rosbag
import rospkg
import PyKDL
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cw3q2.iiwa14DynBase import Iiwa14DynamicBase
from cw3q2.iiwa14DynKDL import Iiwa14DynamicKDL


# from cw3q2.src.cw3q2.iiwa14DynStudent import Iiwa14DynamicRef
kdl = Iiwa14DynamicKDL(Iiwa14DynamicBase)

##TODO: Implement your cw3q5 coding solution here.


rospack = rospkg.RosPack()
path = rospack.get_path('cw3q5')


bag = rosbag.Bag(path + '/bag/cw3q5.bag')

# topics = bag.get_type_and_topic_info()[1].keys()
# types = []
# for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
#     types.append(bag.get_type_and_topic_info()[1].values()[i][0])

joint_data = [] # create list for joint data 
for topic, msg, t in bag.read_messages(topics='/iiwa/EffortJointInterface_trajectory_controller/command'):
    for i in range(3):
        joint_data.append(msg.points[i].positions) #read position from bag and append to the list , 7x3
print(list(joint_data[0]))