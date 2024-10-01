package de.orat.math.xml.urdf.util;

import de.orat.math.xml.urdf.api.Chain.DH;
import de.orat.math.xml.urdf.api.Chain.RPY;
import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Vector3d;

/**
 * https://robotics.stackexchange.com/questions/8516/getting-pitch-yaw-and-roll-from-rotation-matrix-in-dh-parameter
 * 
 * 
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class RotationUtils {

    /**
     * Gets the extrinsic rotation matrix defined by roll about x, then pitch 
     * about y, then yaw about z. 
     * 
     * This is the rotation matrix used in URDF.<p>
     * 
     * These are the Z1-Y2-X3 Tait-Bryan angles.<p>
     *  
     * @param rpy roll, pitch, yaw in [rad]
     *
     * @return rotation A 3x3 double matrix for the resulting extrinsic rotation.
     */
    public static Matrix3d getExtrinsicRotation(RPY rpy){
    
        //Precompute sines and cosines of Euler angles
        double su = Math.sin(rpy.roll());
        double cu = Math.cos(rpy.roll());
        double sv = Math.sin(rpy.pitch());
        double cv = Math.cos(rpy.pitch());
        double sw = Math.sin(rpy.yaw());
        double cw = Math.cos(rpy.yaw());
        
        //Create and populate RotationMatrix
        return new Matrix3d(cv*cw, su*sv*cw - cu*sw, su*sw + cu*sv*cw,
                                  cv*sw, cu*cw + su*sv*sw, cu*sv*sw - su*cw,
                                  -sv, su*cv, cu*cv); 
    }
    
    public static Matrix4d to(RPYXYZ rpyxyz){
        return new Matrix4d(getExtrinsicRotation(rpyxyz.rpy()), rpyxyz.xyz(), 1d);
    }
    /*
    alternative Implementierung aus python file und weitere Ansätze
      //AxisAngle4d x_rot = new AxisAngle4d(1d,0d,0d, roll); //x_rot = x_rotation(rpy[0])
        //AxisAngle4d y_rot = new AxisAngle4d(0d, 1d, 0d, pitch); // y_rotation(rpy[1])
        //AxisAngle4d z_rot = new AxisAngle4d(0d, 0d, 1d, yaw); // z_rotation(rpy[2])
        
        Matrix3d x_rot_m = new Matrix3d();
        //x_rot_m.set(x_rot);
        x_rot_m.rotX(roll);
        Matrix3d y_rot_m = new Matrix3d();
        //y_rot_m.set(y_rot);
        y_rot_m.rotY(pitch);
        Matrix3d z_rot_m = new Matrix3d();
        //z_rot_m.set(z_rot);
        z_rot_m.rotZ(yaw);
        
        y_rot_m.mul(x_rot_m);
        z_rot_m.mul(y_rot_m);
        return z_rot_m;
        // return np.matmul(z_rot, np.matmul(y_rot, x_rot))

    */
    
    /**
     * Get the inverse of a homogeneous transform.
     *
     */
    /*public Matrix4d inv_tf(Matrix4d tf){
        Matrix4d result = new Matrix4d();
        
        inv_tf = np.eye(4)
        inv_tf[0:3, 0:3] = np.transpose(tf[0:3, 0:3])
        inv_tf[0:3, 3] = -1.0 * np.matmul(np.transpose(tf[0:3, 0:3]), tf[0:3, 3])
        return inv_tf
    }*/
    
    /**
     * Get the tf for the given dh parameters.
     *
     * A transformation matrix which transforms coordinates of a link i-1 to 
     * coordinates in the frame of link i.<p>
     * 
     * @param dh Denavit-Hartenberg parameters
     * @return homogenious matrix R_z(theta)*T_z(d)*T_x(a)*R_x(alpha)
     */
    public static Matrix4d to(DH dh){
        Matrix4d result = new Matrix4d();
        result.m00 = Math.cos(dh.theta());
        result.m01 = -Math.sin(dh.theta())*Math.cos(dh.alpha());
        result.m02 = Math.sin(dh.theta())*Math.sin(dh.alpha());
        result.m03 = dh.r()*Math.cos(dh.theta());
         
        result.m10 = Math.sin(dh.theta());
        result.m11 = Math.cos(dh.theta())*Math.cos(dh.alpha());
        result.m12 = -Math.cos(dh.theta())*Math.sin(dh.alpha());
        result.m13 = dh.r()*Math.sin(dh.theta());

        result.m21 = Math.sin(dh.alpha());
        result.m22 = Math.cos(dh.alpha());
        result.m23 = dh.d();
        
        result.m33 = 1d;
        return result;
    }
    /**
     * Create a sequence of two transformation matrices from the DH parameters.
     * 
     * 1. R_z(theta)*T_z(d)
     * 2. T_x(r)*R_x(alpha)
     * 
     * @param dh Denavit-Hartenberg parameters
     * @return a sequence of rotation matrices representing the transformation
     * from link i-1 to link i.
     */
    public static Matrix4d[] to2Step(DH dh){
        Matrix4d[] result = new Matrix4d[2];
        Matrix4d first = new Matrix4d();
        first.rotZ(dh.theta());
        first.set(new Vector3d(0,0,dh.d()));
        Matrix4d second = new Matrix4d();
        second.set(new Vector3d(dh.r(),0,0));
        second.rotX(dh.alpha());
        return result;
    }
    
    /*  def DH_trans(DH, joint_val):

     d, theta, a, alpha = (0,0,0,0)

     if (DH[0] == 'r'):

         d, theta, a, alpha = (DH[1], joint_val, DH[2], DH[3])

     elif (DH[0] == 'p'):

         d, theta, a, alpha = (joint_val, DH[1], DH[2], DH[3])

     elif (DH[0] == 'f'):

         d, theta, a, alpha = (DH[1], DH[2], DH[3], DH[4])

     trans_mat = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha),    a*cos(theta)],
                           [sin(theta), cos(theta)*cos(alpha),    -1*cos(theta)*sin(alpha), a*sin(theta)],
                           [0,          sin(alpha),               cos(alpha),               d           ],
                           [0,          0,                        0,                        1           ]])

     return trans_mat
*/
    
    /**
     * Convert a 3x3 transform matrix to roll-pitch-yaw.
     *
     * The roll-pitch-yaw axes in a typical URDF are defined as a
     * rotation of ``r`` radians around the x-axis followed by a rotation of
     * ``p`` radians around the y-axis followed by a rotation of ``y`` radians
     * around the z-axis. These are the Z1-Y2-X3 Tait-Bryan angles. See
     * Wikipedia_ for more information.<p>
     *
     * https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix<p>
     *
     * There are typically two possible roll-pitch-yaw coordinates that could have
     * created a given rotation matrix. Specify ``secondSolution=false`` for the first one
     * and ``secondSolution=true`` for the second one.<p>
     *
     * @return The roll-pitch-yaw coordinates in order (x-rot, y-rot, z-rot).
   */
    public static RPY toRPY(Matrix3d rot, boolean secondSolution) {    
        double r,p,y = 0;
        if (Math.abs(rot.m20) >= 1.0 - 1e-12){
            if (rot.m20< 0){
                p = Math.PI / 2;
                r = Math.atan2(rot.m01, rot.m02);
            } else{
                p = -Math.PI / 2;
                r = Math.atan2(-rot.m01, -rot.m02);
            }
        } else {
            if (!secondSolution){
                p = -Math.asin(rot.m20);
            } else {
                p = Math.PI + Math.asin(rot.m20);
            }
            r = Math.atan2(rot.m21 / Math.cos(p), rot.m22 / Math.cos(p));
            y = Math.atan2(rot.m10 / Math.cos(p), rot.m00 / Math.cos(p));
        }
        return new RPY(r,p,y);
    }
    
    /**
     * Convert a 4x4 homogenous matrix to rpyxyz representation.
     * 
     * @param The homogenous transform matrix.
     * @param secondSolution defines which of the two possible angle solutions should be used
     */
    public static RPYXYZ toRPYXYZ(Matrix4d tf, boolean secondSolution){
        Matrix3d rot = new Matrix3d();
        tf.getRotationScale(rot);
        RPY rpy = toRPY(rot, secondSolution);
        Vector3d xyz = new Vector3d();
        tf.get(xyz);
        return new RPYXYZ(rpy, xyz);
    }
}
