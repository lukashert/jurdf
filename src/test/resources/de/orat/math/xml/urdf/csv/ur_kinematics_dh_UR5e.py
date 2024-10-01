import math

import numpy as np
from PyKDL import Chain, Segment, Joint, Frame, ChainIkSolverPos_LMA, Rotation, Vector, JntArray
from numpy.linalg import inv

# Transforms for translating between the coordinate conventions of the RPS and those assumed by the kinematics implementation (*_KIN)
TF_FLANGE_TO_TCP = np.array([[1.0000000, 0.0000000, 0.0000000, 0.0],
                             [0.0000000, -1.0000000, -0.0000000, 0.0],
                             [0.0000000, 0.0000000, -1.0000000, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])  # Rotation 180 around X


class URKinematicsDH:
    def __init__(self, robot_type='UR5e', tf_world_to_base=None):
        """ This class allows you to create a forward kinematics model for UR type robots
            It can easily be extended to other robots by changing the DH_table

        Args:
            robot_type (str): It is a string that encodes what robot can be used.
                               Right now only UR5 and UR5e are available
        """
        self.n_joints = 6
        self.tf_world_to_base = np.eye(4) if tf_world_to_base is None else tf_world_to_base

        if 'UR5e' == robot_type:
            # manufacturer's DH params
            self.dh_table = {
                'a': np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]),
                'd': np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]),
                'alpha': np.array([math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0]),
                'theta': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            }
            self.dh_offsets = {
                'a': np.array([0.000156734465764371306, 0.109039760794650886, 0.00135049423466820917, 6.30167176077633267e-05, 8.98147062591837358e-05, 0]),
                'd': np.array([-7.63582045015809285e-05, 136.026368377065324, -143.146527922606964, 7.12049886607637639, -6.13561334270734671e-05, -0.000218168195914358876]),
                'alpha': np.array([-0.000849612070594307767, 0.00209120614311242205, 0.0044565542371754396, -0.000376815598678081898, 0.000480742313784698894, 0]),
                'theta': np.array([-8.27430119976213518e-08, 0.732984551101984239, 5.46919521494736127, 0.0810043775014757245, -3.53724730506321805e-07, -9.97447025669062626e-08])
            }
        self.kdl_chain = self._build_kdl_chain()
        self.ik_solver = ChainIkSolverPos_LMA(self.kdl_chain, maxiter=1000)

    def _build_T(self, ref: int, end: int, theta: np.array) -> np.ndarray:
        """Creates the transformation matrices from ref to end (as long as end=ref+1),
           using the values for the angle theta.

        Args:
            ref (int): is the reference frame
            end (int): is the ending frame
            theta (np.array): is a vector with the values for theta linking the two frames for each data-point

        Returns:
            T (np.ndarray): is the transformation matrix for the two frames for each data-point.
                            The shape is (theta.shape[0], 4, 4).
        """
        if not theta.shape: theta = theta[np.newaxis, np.newaxis]
        theta = theta + self.dh_table['theta'][ref] + self.dh_offsets['theta'][ref]
        alpha = self.dh_table['alpha'][ref] + self.dh_offsets['alpha'][ref]
        a = self.dh_table['a'][ref] + self.dh_offsets['a'][ref]
        d = self.dh_table['d'][end - 1] + self.dh_offsets['d'][ref]
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        T = np.zeros((theta.shape[0], 4, 4))
        T[:, 0, 0] = ct
        T[:, 0, 1] = -st * ca
        T[:, 0, 2] = st * sa
        T[:, 0, 3] = a * ct
        T[:, 1, 0] = st
        T[:, 1, 1] = ct * ca
        T[:, 1, 2] = -ct * sa
        T[:, 1, 3] = a * st
        T[:, 2, 0] = 0.0
        T[:, 2, 1] = sa
        T[:, 2, 2] = ca
        T[:, 2, 3] = d
        T[:, 3, 3] = 1.0
        return T

    def _build_kdl_chain(self) -> Chain:
        chain = Chain()
        for i in range(self.n_joints):
            a = self.dh_table["a"][i] + self.dh_offsets['a'][i]
            alpha = self.dh_table["alpha"][i] + self.dh_offsets['alpha'][i]
            d = self.dh_table["d"][i]  + self.dh_offsets['d'][i]
            theta = self.dh_table["theta"][i] + self.dh_offsets['theta'][i]
            chain.addSegment(Segment(Joint(Joint.RotZ), Frame.DH(a, alpha, d, theta)))
        return chain

    def forward(self, theta: np.ndarray) -> np.ndarray:
        """Generates N (N is the number of data-points) concatenated transformation matrices for each datapoint,
           using the angles values provided by theta

        Args:
            theta (np.ndarray): is the angle values for the robot. When the robot has n_joints=6 then the shape=(N, n_joints),
            where N is the number of data-points. If you have only one data-point, make sure that the shape is (1, n_joints) and
            not (n_joints)

        Returns:
            np.ndarray: Returns the full transformation matrix for the UR model. The shape=(N, 4, 4), where N is the number of
            data-points.
        """
        theta_ = np.expand_dims(theta, 0) if len(theta.shape) == 1 else theta
        T = np.einsum('ij, k-> kij', np.eye(4, dtype=np.float64), np.ones(theta_.shape[0]))
        for i in range(self.n_joints):
            T = np.einsum('...ij,...jk->...ik', T, self._build_T(i, i + 1, theta_[..., i]))
        base_to_tcp = T @ TF_FLANGE_TO_TCP
        world_to_tcp = self.tf_world_to_base @ base_to_tcp
        world_to_tcp = world_to_tcp.squeeze() if len(theta.shape) == 1 else world_to_tcp
        return world_to_tcp

    def inverse(self, tcp_world, theta_ref=np.zeros(6)):
        """
        :param tcp_world: 4x4 homogeneous transformation matrix with TCP pose in world coordinates
        :param theta_ref: Reference joint state. The solution closest to this state is returned
        """
        base_to_tcp = inv(self.tf_world_to_base) @ tcp_world
        base_to_flange = base_to_tcp @ inv(TF_FLANGE_TO_TCP)
        T = base_to_flange

        q_out = JntArray(len(theta_ref))
        q_init = JntArray(len(theta_ref))
        for i in range(len(theta_ref)):
            q_out[i] = 0.0
            q_init[i] = theta_ref[i]
        res = self.ik_solver.CartToJnt(q_init, Frame(Rotation(*T[:3, :3].flatten()),
                                                     Vector(*T[:3, 3])), q_out)
        if res != 0:
            raise RuntimeError("Could not find IK solution")
        return np.array([q_out[i] for i in range(len(theta_ref))])
