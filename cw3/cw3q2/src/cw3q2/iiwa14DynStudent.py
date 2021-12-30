#!/usr/bin/env python

import numpy as np
from iiwa14DynBase import Iiwa14DynamicBase


class Iiwa14DynamicRef(Iiwa14DynamicBase):
    def __init__(self):
        super(Iiwa14DynamicRef, self).__init__(tf_suffix='ref')

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        joint. Reference Lecture 9 slide 13.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 7.
        Returns:
            np.ndarray The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to
            the selected joint frame.
        """

        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)

        T = np.identity(4)
        # iiwa base offset
        T[2, 3] = 0.1575

        # 1. Recall the order from lectures. T_rot_z * T_trans * T_rot_x * T_rot_y. You are given the location of each
        # joint with translation_vec, X_alpha, Y_alpha, Z_alpha. Also available are function T_rotationX, T_rotation_Y,
        # T_rotation_Z, T_translation for rotation and translation matrices.
        # 2. Use a for loop to compute the final transformation.
        for i in range(0, up_to_joint):
            T = T.dot(self.T_rotationZ(joints_readings[i]))
            T = T.dot(self.T_translation(self.translation_vec[i, :]))
            T = T.dot(self.T_rotationX(self.X_alpha[i]))
            T = T.dot(self.T_rotationY(self.Y_alpha[i]))

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"

        return T

    def get_jacobian_centre_of_mass(self, joint_readings, up_to_joint=7):
        """Given the joint values of the robot, compute the Jacobian matrix at the centre of mass of the link.
        Reference - Lecture 9 slide 14.

        Args:
            joint_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute the Jacobian.
            Defaults to 7.

        Returns:
            jacobian (numpy.ndarray): The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the
            centre of mass of a link.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7

        # Your code starts here ----------------------------
        jacobian = np.zeros((6,7))

        # zjminus1 = []
        # Pli = []
        # Pjminus1 = []
        # Tl = self.forward_kinematics_centre_of_mass(joint_readings, 7) 
        # Pli = Tl[0:3,3]

        for i in range(up_to_joint):
            Tj = self.forward_kinematics(joint_readings, i)
            # Pjminus1.append(Tj[0:3,3])
            # zjminus1.append(Tj[0:3,2])
            Tl = self.forward_kinematics_centre_of_mass(joint_readings, i+1) 
            # Pli.append(Tl[0:3,3])
            Pli = Tl[0:3,3]
        # for i in range(up_to_joint):
            Pjminus1 = Tj[0:3,3] # position vector of the origin of frame j-1
            zjminus1 = Tj[0:3,2] # unit vector of axis z of frame j-1 
            jacobian[0:3,i] = np.cross(zjminus1 , (Pli - Pjminus1))
            jacobian[3:6,i] = zjminus1 
        # Your code ends here ------------------------------

        assert jacobian.shape == (6, 7)
        return jacobian

    def forward_kinematics_centre_of_mass(self, joints_readings, up_to_joint=7):
        """This function computes the forward kinematics up to the centre of mass for the given joint frame.
        Reference - Lecture 9 slide 14.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint} for the
            centre of mass w.r.t the base of the robot.
        """
        T= np.identity(4)
        T[2, 3] = 0.1575

        T = self.forward_kinematics(joints_readings, up_to_joint-1)
        T = T.dot(self.T_rotationZ(joints_readings[up_to_joint-1]))
        T = T.dot(self.T_translation(self.link_cm[up_to_joint-1, :]))

        return T

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        B = np.zeros((7, 7))
        
	# Your code starts here ------------------------------
        
        for i in range(7):
            Ioili = np.eye(3)
            Ioili[0,0] = self.Ixyz[i][0]
            Ioili[1,1] = self.Ixyz[i][1]
            Ioili[2,2] = self.Ixyz[i][2]
            R0G = self.forward_kinematics_centre_of_mass(joint_readings, i+1)[0:3,0:3]
            Jli = np.dot(np.dot(R0G,Ioili), R0G.T)
            mass = self.mass[i]
            jacobian = self.get_jacobian_centre_of_mass(joint_readings, i+1)
            Jp = jacobian[0:3,:]
            Jo = jacobian[3:6,:]
            B += mass*np.dot(Jp.T,Jp)+ np.dot(Jo.T,np.dot(Jli, Jo))

    # Your code ends here ------------------------------
        
        return B

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        assert isinstance(joint_velocities, list)
        assert len(joint_velocities) == 7

        # Your code starts here ------------------------------
        h = [0.000001]*7
        b = self.get_B(joint_readings)
        cij = np.zeros((7,7))
        C = np.zeros((7,))
        for i in range(7):
            for j in range(7):
                for k in range(7):
                    qk = joint_readings
                    qi = joint_readings
                    bij = self.get_B(list(np.add(qk,h)))
                    bjk = self.get_B(list(np.add(qi,h)))
                    bij_deri = (bij[i,j]-b[i,j])/h[0]
                    bjk_deri = (bjk[j,k]-b[j,k])/h[0]
                    hijk = bij_deri - 0.5*  bjk_deri
                    cij[i,j] += hijk * joint_velocities[k]  
        C = np.dot(cij, joint_velocities).reshape(7,)

        # Your code ends here ------------------------------

        assert isinstance(C, np.ndarray)
        assert C.shape == (7,)
        return C

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            G (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7

        # Your code starts here ------------------------------
        g = np.zeros((7,))
        g0T = np.array([0,0,-self.g]).reshape(3,1)
        Pq = 0
        Pq_h = 0
        h = 0.000001
        for i in range(7):
            Tl = self.forward_kinematics_centre_of_mass(joint_readings, i+1) 
            Pli = Tl[0:3,3]
            Pq -= self.mass[i]*np.dot(g0T.T, Pli)

            Tl_h = self.forward_kinematics_centre_of_mass(list(np.add(joint_readings , h)), i+1) 
            Pli_h = Tl_h[0:3,3]
            Pq_h -= self.mass[i]*np.dot(g0T.T, Pli_h)

            g_deri = (Pq_h - Pq)/h
            g[i] = g_deri
        # Your code ends here ------------------------------

        assert isinstance(g, np.ndarray)
        assert g.shape == (7,)
        return g
