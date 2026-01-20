from pyopensot.tasks.velocity import Postural, Cartesian
from xbot2_interface import pyxbot2_interface as xbi
import numpy as np
from enum import Enum

"""
Simple Inverse Kinematics solver using OpenSot with a Cartesian task as primary task and postural task as secondary task.
"""
class inverse_kinematics:
    def __init__(self, urdf, q):
        # Create XBot2 model using URDF
        self.model = xbi.ModelInterface2(urdf)

        # Set q as homing configuration
        self.model.setJointPosition(q)
        self.model.update()

        # Create a Cartesian task at frame fp3_link8, set lambda gain
        self.task1 = Cartesian("Cartesian", self.model, "fp3_link8", "world")
        self.task1.setLambda(0.1)
        self.task1.update()

        self.pose_ref, self.vel_ref = self.task1.getReference()

        # Create a postural task as secondary task
        self.task2 = Postural(self.model)
        self.task2.setLambda(0.01)
        self.task2.update()

        self.q_ref, self.qdot_ref = self.task2.getReference()


    def update(self, q):
        # Update actual position in the model
        self.model.setJointPosition(q)
        self.model.update()

        # Update tasks with references
        self.task1.setReference(self.pose_ref, self.vel_ref)
        self.task1.update()

        # Update postural task reference
        self.task2.setReference(self.q_ref)
        self.task2.update()

    def solve(self, reg=1e-3):
        # get task Jacobian  and compute inverse
        J = self.task1.getA()
        J_pinv = self.damped_pinv(J, reg)


        # get Cartesian error and compute dq
        e = self.task1.getb()
        dq = J_pinv @ e

        # project postural task into null space
        dq += self.null_space_projector(J, J_pinv) @ self.task2.getb()

        return dq

    """
    Damped pseudoinverse computation
    """
    def damped_pinv(self, A, reg):
        U, S, Vt = np.linalg.svd(A)

        S_inv = np.zeros((A.shape[1], A.shape[0]))
        for i in range(len(S)):
            S_inv[i, i] = S[i] / (S[i]**2 + reg**2)

        return Vt.T @ S_inv @ U.T

    """
    Null space projector computation: N = I - A_pinv * A
    """
    def null_space_projector(self, A, A_pinv):
        return np.eye(A.shape[1]) - A_pinv @ A
