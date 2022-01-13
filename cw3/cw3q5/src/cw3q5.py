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
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import matplotlib.pyplot as plt

class iiwaDynamics(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('iiwa_cw3', anonymous=True)
        self.kdl_iiwa = Iiwa14DynamicKDL()
        
        self.previous_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.previous_t = 0

        self.acc = [] #save acceleration into a list
        self.time = []
        self.joint_name = []
        # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
        self.traj_pub = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                queue_size=5)
        self.checkpoint_pub = rospy.Publisher("/iiwa/checkpoint_positions", Marker, queue_size=100)
        
        self.sub_dynamics = rospy.Subscriber('/iiwa/joint_states', JointState, self.callback, queue_size=5)
                                             
        self.pose_broadcaster = TransformBroadcaster()

        # self.B = 0
        # self.Cxqdot = 0
        # self.G = 0
        # self.Tau = 0

    def load_targets(self):
        """
        This fuction loads the bag file and returns the list of joint position and joint names

        """

        rospack = rospkg.RosPack()
        path = rospack.get_path('cw3q5')

        bag = rosbag.Bag(path + '/bag/cw3q5.bag')

        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

        joint_data = [] # create list for joint data 
        time_from_start = [] #  create list for time from start
        for topic, msg, t in bag.read_messages(topics='/iiwa/EffortJointInterface_trajectory_controller/command'):
            for i in range(3):
                joint_data.append(msg.points[i].positions) #read position from bag and append to the list , 7x3
                time_from_start.append(msg.points[i].time_from_start.secs)
            joint_names = msg.joint_names

        joint_data = map(list,joint_data)

        print("\nBag topic: {}\n".format(topics))
        print("Message type: {}\n".format(types))
        print("Joint Names: {}\n".format(joint_names))
        print("Joint Positions: {}\n".format(joint_data))
        # Close the bag
        bag.close()

        return joint_data, joint_names, time_from_start

    def run(self):
        """This function is the main run function of the class. When called, it runs question 6 by calling the q6()
        function to get the trajectory. Then, the message is filled out and published to the /command topic.
        """
        print("running trajectory")
        # rospy.sleep(2.0)
        joint_data, joint_names, time_from_start = self.load_targets()
        self.joint_name.append(joint_names)
        self.final_pos = joint_data[-1]
        rospy.loginfo("Waiting 5 seconds for everything to load up.")
        rospy.sleep(2.0)
        # sub_dynamics = rospy.Subscriber('/iiwa/joint_states', JointState, self.callback, queue_size=5)
        traj = self.trajectory(joint_data, time_from_start)
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = joint_names
        self.traj_pub.publish(traj)
        print("publishing trajectory")
        
    def trajectory(self, joint_data, time_from_start):
        
        traj = JointTrajectory()

        # this is purely just for visualisation
        checkpoints = np.zeros((4, 4, len(joint_data)), dtype=float) # 4x4x3
        for i in range(len(joint_data)):
            checkpoints[:, :, i] = self.kdl_iiwa.forward_kinematics(joint_data[i])
       
        self.publish_checkpoints(checkpoints)
        print("publishing checkpoints")
        # t = 2
        # dt = 2
        for i in range(len(joint_data)):
            traj_point = JointTrajectoryPoint()
            traj_point.positions = joint_data[i] # joint position for each point
            traj_point.time_from_start.secs = time_from_start[i] # load time from start for point
            # t = t + dt
            # traj_point.time_from_start.secs = t
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

    def callback(self, msg):
        current_joint_position = msg.position
        current_pose = self.kdl_iiwa.forward_kinematics(current_joint_position)
        self.broadcast_pose(current_pose, 'ee_kdl') 

        current_t = msg.header.stamp.secs + msg.header.stamp.nsecs * np.power(10.0, -9)
        joint_velocities = list((np.array(current_joint_position) - np.array(self.previous_joint_position)) / (current_t - self.previous_t))
        kdl_joint_velocities = msg.velocity
        self.previous_joint_position = current_joint_position
        self.previous_t = current_t
        Tau = list(msg.effort)

        B = self.kdl_iiwa.get_B(current_joint_position)
        Cxqdot = self.kdl_iiwa.get_C_times_qdot(current_joint_position, joint_velocities)
        G = self.kdl_iiwa.get_G(current_joint_position)

        
        # acc = []
        position_dif = np.array(self.final_pos)- np.array(current_joint_position)
        norm = np.linalg.norm(position_dif)
        acceleration = self.acceleration(B, Cxqdot, G, Tau)
        self.acc.append(acceleration)
        self.time.append(rospy.Time.now().to_sec())
        print(rospy.Time.now().to_sec(), norm)
            # print(acceleration)
        # print(np.array(acceleration))

        if norm < 0.0315:   
            print("Final Position Reached")
            rospy.signal_shutdown("Final Position Reached")
            # acc_plot = self.plot_acceleration()
            

        # print('Acceleration: {}\n'.format(acceleration))

        # print('velocity: {}'.format(joint_velocities))
        # print('KDL velocity: {}'.format(list(kdl_joint_velocities)))
        # while 
        # while position_dif 

        # print(rospy.Time.now())

        # self.acc.append(acceleration)

    def acceleration(self, B, Cxqdot, G, Tau):
        
        eq = np.array(Tau) - np.array(Cxqdot) - np.array(G)
        acc = np.dot(np.linalg.inv(B) , eq)

        return acc

    def plot_acceleration(self):

        print('Plotting Graph')

        acc_list = np.squeeze(self.acc)
        
        # print(np.squeeze(self.acc))
        acc_j1 = [item[0] for item in acc_list]
        acc_j2 = [item[1] for item in acc_list]
        acc_j3 = [item[2] for item in acc_list]
        acc_j4 = [item[3] for item in acc_list]
        acc_j5 = [item[4] for item in acc_list]
        acc_j6 = [item[5] for item in acc_list]
        acc_j7 = [item[6] for item in acc_list]

        time = np.squeeze(self.time)
        joint_name = np.squeeze(self.joint_name)
        plt.plot(time, acc_j1, 'orange', label=joint_name[0])
        plt.plot(time, acc_j2, 'm', label=joint_name[1])
        plt.plot(time, acc_j3, 'r', label=joint_name[2])
        plt.plot(time, acc_j4, 'g', label=joint_name[3])
        plt.plot(time, acc_j5, 'y', label=joint_name[4])
        plt.plot(time, acc_j6, 'c', label=joint_name[5])
        plt.plot(time, acc_j7, 'b', label=joint_name[6])
        plt.xlabel("Time (s)")
        plt.ylabel("Joint Acceleration ($rad/s^2$)")
        plt.title("Joint Acceleration of iiwa14 manipulator")
        plt.legend(loc="upper right")
        plt.show()
        plt.savefig('cw3q5_plot.png')

    def broadcast_pose(self, pose, suffix):
        """Given a pose transformation matrix, broadcast the pose to the TF tree.
        Args:
            pose (np.ndarray): Transformation matrix of pose to broadcast.

        """
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'iiwa_ee_' + suffix

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    @staticmethod
    def rotmat2q(T):
        """Convert rotation matrix to Quaternion.
        Args:
            T (np.ndarray): Rotation matrix to convert to Quaternion representation.

        Returns:
            q (Quaternion): Quaternion conversion of given rotation matrix.
        """
        q = Quaternion()

        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        xr = T[2, 1] - T[1, 2]
        yr = T[0, 2] - T[2, 0]
        zr = T[1, 0] - T[0, 1]

        x = xr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        y = yr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        z = zr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

        q.w = np.cos(angle / 2)
        q.x = x * np.sin(angle / 2)
        q.y = y * np.sin(angle / 2)
        q.z = z * np.sin(angle / 2)
        return q

if __name__ == '__main__':
    try:
        iiwa_dynamics = iiwaDynamics()
        iiwa_dynamics.run()
        rospy.spin()
        rospy.on_shutdown(iiwa_dynamics.plot_acceleration)
    except rospy.ROSInterruptException:
        pass
