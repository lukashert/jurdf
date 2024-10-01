package de.orat.math.xml.urdf.util.spatialAlgebra;

import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Utils {
    
    /**
     * The motion cross product matrix of a spatial vector.
     * 
     * @param v spatial vector
     * @return motion cross product matrix
     */
    public Matrix6d createMotionCrossProduct(Vector6d v){
   
        //mcp = cs.SX.zeros(6, 6)
        Matrix6d mcp = new Matrix6d();

        mcp.setElement(0, 1, -v.getElement(2));
        mcp.setElement(0, 2,  v.getElement(1));
        mcp.setElement(1, 0, v.getElement(2));
        mcp.setElement(1, 2, -v.getElement(0));
        mcp.setElement(2, 0, -v.getElement(1));
        mcp.setElement(2, 1, v.getElement(2));

        mcp.setElement(3, 4, -v.getElement(2));//[2]
        mcp.setElement(3, 5, v.getElement(1));//[1]
        mcp.setElement(4, 3, v.getElement(2));//[2]
        mcp.setElement(4, 5, -v.getElement(0));//[0]
        mcp.setElement(5, 3, -v.getElement(1));//[1]
        mcp.setElement(5, 4, v.getElement(0));//[0]

        mcp.setElement(3, 1, -v.getElement(5));//[5]
        mcp.setElement(3, 2, v.getElement(4));//[4]
        mcp.setElement(4, 0, v.getElement(5));//[5]
        mcp.setElement(4, 2, -v.getElement(3));//[3]
        mcp.setElement(5, 0, -v.getElement(4));//[4]
        mcp.setElement(5, 1, v.getElement(3));//[3]

        return mcp;
    }
    
    /**
     * The force cross product matrix of a spatial vector.
     * 
     * @param v
     * @return 
     */
    public Matrix6d createForceCrossProduct(Vector6d v){
        Matrix6d result = createMotionCrossProduct(v);
        result.negate();
        result.transpose();
        return result;
    }
    
    /**
     * A rotation matrix from roll pitch yaw. ZYX convention..
     * 
     * @param roll
     * @param pitch
     * @param yaw
     * @return 
     */
    public Matrix3d createRotationRPY(double roll, double pitch, double yaw){
        double cr = Math.cos(roll);
        double sr = Math.sin(roll);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);
        return new Matrix3d(cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
                             sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
                            -sp,             cp*sr,             cp*cr);
    }
    
    
    /*def spatial_transform(R, r):
    """Returns the spatial motion transform from a 3x3 rotation matrix
    and a 3x1 displacement vector."""
    X = cs.SX.zeros(6, 6)
    X[:3, :3] = R
    X[3:, 3:] = R
    X[3:, :3] = -cs.mtimes(R, cs.skew(r))
    return X
    
    invers
    X = cs.SX.zeros(6, 6)
    X[:3, :3] = R.T
    X[3:, 3:] = R.T
    X[3:, :3] = cs.mtimes(cs.skew(r), R.T)
     */        
    /**
     * The spatial motion transform from a 3x3 rotation matrix
     * and a 3x1 displacement vector.
     * 
     * @param R
     * @param r 
     */
    public Matrix6d createSpatialMotionTransform(Matrix3d R_, Vector3d r, boolean inverse){
        Matrix6d result = new Matrix6d();
        Matrix3d R = R_;
        if (inverse) {
            R = new Matrix3d(R_);
            R.transpose();
        }
        result.set(0,0, R);
        result.set(3,3, R);
        Matrix3d m = new Matrix3d();
        m.mul(R, new SkewMatrix3d(r));
        if (!inverse){
            m.mul(-1);
        }
        result.set(3,0, m);
        return result;
    }
    
    /**
     * The spatial transform from child link to parent link with a revolute 
     * connecting joint.
     */
    /*public static Matrix6d createXJTRevolute(xyz, rpy, Vector3d axis, qi, boolean inverse){
        T = tm.revolute(xyz, rpy, axis, qi)
        rotation_matrix = T[:3, :3]
        displacement = T[:3, 3]
        return createSpatialMotionTransform(rotation_matrix.T, displacement, inverse)
    }*/

/*def XJT_revolute_BA(xyz, rpy, axis, qi):
    """Returns the spatial transform from parent link to child link with
    a revolute connecting joint."""
    T = tm.revolute(xyz, rpy, axis, qi)
    rotation_matrix = T[:3, :3]
    displacement = T[:3, 3]
    return spatial_transform_BA(rotation_matrix, displacement)


def XJT_prismatic(xyz, rpy, axis, qi):
    """Returns the spatial transform from child link to parent link with
    a prismatic connecting joint."""
    T = tm.prismatic(xyz, rpy, axis, qi)
    rotation_matrix = T[:3, :3]
    displacement = T[:3, 3]
    return spatial_transform(rotation_matrix.T, displacement)


def XJT_prismatic_BA(xyz, rpy, axis, qi):
    """Returns the spatial transform from parent link to child link with
    a prismatic connecting joint."""
    T = tm.prismatic(xyz, rpy, axis, qi)
    rotation_matrix = T[:3, :3]
    displacement = T[:3, 3]
    return spatial_transform_BA(rotation_matrix, displacement)

*/
}
