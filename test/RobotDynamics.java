package de.orat.math.xml.urdf.util;

/**
 * Computes joint torques.
 *
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class RobotDynamics {

    private RobotModel robotModel;
    private StaticFrictionTypes staticFrictionConfig;
    private ViscousFrictionTypes viscousFrictionConfig;
    
    // 0,1,2
    enum StaticFrictionTypes {
        NONE, STATIC_SIGN, STATIC_TANH;
    }

    enum ViscousFrictionTypes {
        NONE, VISCOUS_LINEAR, VISCOUS_NON_LINEAR;
    }

    /**
     * Initialize a RobotDynamics object.
     *
     * @param robotModel (MotionGroupModel): The motion group model of the robot.
     * @param static_friction_config (StaticFrictionTypes, optional): The configuration for static friction.
     *           Defaults to StaticFrictionTypes.staticSign.
     * @param viscous_friction_config (ViscousFrictionTypes, optional): The configuration for viscous friction.
     *           Defaults to ViscousFrictionTypes.viscousLinear.
     */
    public RobotDynamics(RobotModel robotModel,
        StaticFrictionTypes staticFrictionConfig,
        ViscousFrictionTypes viscousFrictionConfig){
        
        this.robotModel = robotModel;
        this.robotParams = getCurrentStaticRobotParams(robotModel);
        this.staticFrictionConfig = staticFrictionConfig;
        this.viscousFrictionConfig = viscousFrictionConfig;
    }
    
    record SpaceJoint(double[][] position, double[][] velocity, double[][] joint_path.acceleration){};
    
    /**
      *  Computes the torque for a given joint path.
      *
        Args:
            joint_path (SpaceJointObject): The joint path for which to compute the torque.
            implementation (Literal["lagrange cpp", "lagrange python", "newton-euler"], optional):
                The implementation method to use. Defaults to "lagrange cpp".

        @return the computed torque (Union[np.ndarray, Any, np.ndarray[Any, np.dtype[np.float64]]])

        Raises:
            NotImplementedError: If the specified implementation method is not supported.
    */
    // SpaceJointObject
    public void computeTorques(SpaceJoint joint_path){
        //implementation: Literal["lagrange cpp", "lagrange python", "newton-euler"] = "lagrange cpp",
        //  -> Union[np.ndarray, Any, np.ndarray[Any, np.dtype[np.float64]]]:
        
        double[][] q = joint_path.position;
        double[][] qd = joint_path.velocity;
        double[][] qdd = joint_path.acceleration;

        return lagrangianFormalism(q, qd, qdd);
    }
    
    /**
     *  Compute dynamics via lagrange formulation
     *
     *  @return Joint torque
     *   
     */
    public double[] lagrangianFormalism(double[] q, double[] qd, double[] qdd){
       
        num_samples = qdd.shape[0]

        double[][] M = inertia(q);
        double[][] C = coriolis(q, qd);
        double[][] P = gravload(q);
        staticFriction = staticFriction(qd);
        viscousFriction = viscousFriction(qd);

        double[] tau = (
            np.matmul(M, np.reshape(qdd, (num_samples, len(self.robot_params), 1)))
            + np.matmul(C, np.reshape(qd, (num_samples, len(self.robot_params), 1)))
            + np.reshape(P, (num_samples, len(self.robot_params), 1))
            + np.reshape(static_friction, (num_samples, len(self.robot_params), 1))
            + np.reshape(viscous_friction, (num_samples, len(self.robot_params), 1))
        )
        return np.reshape(tau, (num_samples, len(self.robot_params)))

    
    def calc_T(self, q: np.ndarray) -> dict:
        """
        Compute transformation matrix of the dh ("T_dh") and the dh with a center of mass offset ('T_dh_with_com')

        :param q: joint position
        :type q: ndarray(n) or ndarray(m,n)

        :return: Transformationmatrix
        :rtype: dict{"T_dh": ndarray(n, 4, 4) or ndarray(m, n, 4, 4), "T_dh_with_com": ndarray(n, 4, 4) or ndarray(m, n, 4, 4)}

        'calc_T(q)' where the argument have shape (n) where n is the number of robot joints.
        The result has shape (n, 4, 4).

        'calc_T(q)' where the arguments have shape (m, n) where n is the number of robot joints and m is a joint state.
        The result has shape (m, n, 4, 4).

        .. note::
            - This is a pure Python implementation
        """
        q = setMatrix(q)
        T_dh = np.zeros((q.shape[0], len(self.robot_params), 4, 4))
        T_dh_with_com = np.zeros_like(T_dh)
        for i, robot_param in enumerate(self.robot_params):
            T_dh[:, i] = revoluteTMatrix(q[:, i], robot_param.dh)

            T_dh_with_com[:, i] = np.matmul(
                T_dh[:, i],
                np.array(
                    [
                        [1, 0, 0, robot_param.link.centerOfMass.x],
                        [0, 1, 0, robot_param.link.centerOfMass.y],
                        [0, 0, 1, robot_param.link.centerOfMass.z],
                        [0, 0, 0, 1],
                    ]
                ),
            )
        return {"T_dh": T_dh, "T_dh_with_com": T_dh_with_com}

    def jacobi(self, q: np.ndarray) -> np.ndarray:
        """
        Compute jacobi matrix of the links
        TODO: wrap C++ implementation?

        :param q: joint position
        :type q: ndarray(n) or ndarray(m,n)

        :return: Jacobi matrix
        :rtype: ndarray(n, 6, n) or ndarray(n, 6, n)

        'jacobi(q)' where the argument have shape (n) where n is the number of robot joints.
        The result has shape (n, 6, n).

        'jacobi(q)' where the arguments have shape (m, n) where n is the number of robot joints and m is a joint state.
        The result has shape (m, n, 6, n).

        .. note::
            - This is a pure Python implementation
        """
        dof = len(self.robot_params)

        q = setMatrix(q)
        T_i_j = self.calc_T(q)
        J = np.zeros((T_i_j["T_dh_with_com"].shape[0], dof, 6, dof))
        T_0_j = np.array(calcT_0_j(T_i_j["T_dh"], T_i_j["T_dh_with_com"], dof))
        for i in range(dof):
            for j in range(i + 1):
                T_j_n = calcT_j_i(T_i_j["T_dh"], T_i_j["T_dh_with_com"][:, i], j, i + 1)
                n = np.reshape(T_0_j[j, :, :3, 0], (T_0_j[j].shape[0], 3, 1))
                o = np.reshape(T_0_j[j, :, :3, 1], (T_0_j[j].shape[0], 3, 1))
                a = np.reshape(T_0_j[j, :, :3, 2], (T_0_j[j].shape[0], 3, 1))
                x = np.reshape(T_j_n[:, 0, 3], (T_j_n.shape[0], 1, 1))
                y = np.reshape(T_j_n[:, 1, 3], (T_j_n.shape[0], 1, 1))

                J[:, i, 3:, j] = np.squeeze(a)
                J[:, i, :3, j] = np.squeeze(o * x - n * y)
        return J

    def dR_dqi(self, q) -> np.ndarray:
        dof = len(self.robot_params)

        T_i_j = self.calc_T(q)
        T_0_j = np.array(calcT_0_j(T_i_j["T_dh"], T_i_j["T_dh_with_com"], dof))
        dR_dq = np.zeros((T_i_j["T_dh"].shape[0], dof, dof, 3, 3))
        for i in range(dof):
            R_i_k = T_i_j["T_dh"][:, i, :3, :3]
            dR_dqi = np.cross(
                np.tile(np.array([[0], [0], [1]]), (R_i_k.shape[0], 1, 1)),
                R_i_k,
                axis=1,
            )
            for j in range(i + 1):
                if i == j:
                    dR_dq[:, i, j] = np.matmul(T_0_j[i][:, :3, :3], dR_dqi)
                else:
                    dR_dq[:, i, j] = np.matmul(dR_dq[:, i - 1, j], R_i_k)
        return dR_dq

    def hessian(self, q):
        """
        Compute hessian matrix of the links

        :param q: joint position
        :type q: ndarray(n) or ndarray(m,n)

        :return: hessian matrix
        :rtype: ndarray(n, n, 6, n) or ndarray(m, n, n, 6, n)

        'hessian(q)' where the argument have shape (n) where n is the number of robot joints.
        The result has shape (n, n, 6, n).

        'hessian(q)' where the arguments have shape (m, n) where n is the number of robot joints and m is a joint state.
        The result has shape (m, n, n, 6, n).

        .. note::
            - This is a pure Python implementation
        """
        dof = len(self.robot_params)
        q = setMatrix(q)
        J = self.jacobi(q)
        H = np.zeros([len(J), dof, dof, 6, dof])
        for k, _ in enumerate(self.robot_params):
            J0 = J[:, k]
            for i in range(dof):
                for j in range(0, i):
                    if k < j:
                        continue
                    else:
                        H[:, k, j, 3:, i] = RobotDynamicss(J0[:, 3:, j], J0[:, 3:, i])k < j:
                        continue
                    else:
                        H[:, k, j, :3, i] = RobotDynamicss(J0[:, 3:, min(i, j))
}
