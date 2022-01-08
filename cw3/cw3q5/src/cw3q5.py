#!/usr/bin/env python

import numpy as np
import rospy
import rosbag
import rospkg
import PyKDL
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
##TODO: Implement your cw3q5 coding solution here.

class iiwaDynamics:
    def __init__(self):
        # Initialize node
        rospy.init_node('youbot_traj_cw2', anonymous=True)

        # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
        # self.traj_pub = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
        #                                 queue_size=5)
        self.traj_pub = rospy.Publisher('', JointTrajectory,
                                queue_size=5)
    

    def load_targets(self):
        """
        This fuction lods the bag file
        """

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

        # Close the bag
        bag.close()

        return list(joint_data)


if __name__ == '__main__':
    try:
        iiwa_dynamics = iiwaDynamics()
        iiwa_dynamics.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
