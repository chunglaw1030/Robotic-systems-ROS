#!/usr/bin/env python
##TODO: Implement your cw3q5 coding solution here.
import numpy as np
import rospy
import rosbag
import rospkg
import PyKDL
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cw3q2.iiwa14DynBase import Iiwa14DynamicBase
from cw3q2.iiwa14DynKDL import Iiwa14DynamicKDL
from visualization_msgs.msg import Marker

class iiwaDynamics(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('iiwa_cw3', anonymous=True)

        # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
        # self.traj_pub = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
        #                                 queue_size=5)
        self.traj_pub = rospy.Publisher('EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                queue_size=7)
        self.checkpoint_pub = rospy.Publisher("checkpoint_positions", Marker, queue_size=100)
        self.kdl_iiwa = Iiwa14DynamicKDL()

    def load_targets(self):
        """
        This fuction loads the bag file and returns the list of joint position and joint names

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
            # joint_names.append(msg.joint_names)
            joint_names = msg.joint_names
        joint_data = map(list,joint_data)


        # Close the bag
        bag.close()

        return joint_data, joint_names

    def run(self):
        """This function is the main run function of the class. When called, it runs question 6 by calling the q6()
        function to get the trajectory. Then, the message is filled out and published to the /command topic.
        """
        print("running trajectory")
        rospy.loginfo("Waiting 5 seconds for everything to load up.")
        rospy.sleep(2.0)
        joint_data, joint_names = self.load_targets()

        rospy.sleep(2.0)

        traj = self.trajectory(joint_data)
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = joint_names
        self.traj_pub.publish(traj)
        print("publishing trajectory")

    def trajectory(self, joint_data):
        
        traj = JointTrajectory()

        checkpoints = np.zeros((4, 4, len(joint_data)), dtype=float) # 4x4x3
        for i in range(len(joint_data)):
            checkpoints[:, :, i] = self.kdl_iiwa.forward_kinematics(joint_data[i])
       
        self.publish_checkpoints(checkpoints)
        print("publishing checkpoints")

        dt = 2
        t = 10
        for i in range(len(joint_data)):
            traj_point = JointTrajectoryPoint()
            traj_point.positions = joint_data[i]
            t = t + dt
            traj_point.time_from_start.secs = t
            traj.points.append(traj_point)
        
        # Your code ends here ------------------------


        assert isinstance(traj, JointTrajectory)
        return traj


    def publish_checkpoints(self, tfs):
        """This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
        Cartesian path of the robot end-effector.
        Args:
            tfs (np.ndarray): A array of 4x4xn homogenous transformations specifying the end-effector trajectory.
        """
        id = 0
        for i in range(0, tfs.shape[2]):
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.b = 0.0
            marker.color.g = 0.0 + id * 0.05
            marker.color.r = 1.0 - id * 0.05
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = tfs[0, -1, i]
            marker.pose.position.y = tfs[1, -1, i]
            marker.pose.position.z = tfs[2, -1, i]
            self.checkpoint_pub.publish(marker)


if __name__ == '__main__':
    try:
        iiwa_dynamics = iiwaDynamics()
        iiwa_dynamics.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
