package de.orat.math.xml.urdf.util;

import de.orat.math.xml.urdf.api.Chain;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Vector3d;

/**
 * Tools to determine Denavit-Hartenberg parameters from the rpy-xyz representation
 * as used in urdf-files.
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class DHUtils {
    
    /**
     * The URDF joint axis with respect to the DH parent frame’s z-axis.
     * 
     * can be:<p>
     * 
     * 1. Collinear<br>
     * 2. Parallel<br>
     * 3. Intersecting<br>
     * 4. Skew<p>
     * 
     * @see get_joint_dh_params<p>
     *
     * @param rel_link_frame link to parent link frame (tf from link frame to parent dh frame)
     * @param axis parent joint axis transformed into the current link frame 
     * (this different to the next joint axis which is rotated by alpha)
     * @return a Denavit-Hartenberg parameters segment
     */
    public static Chain.DH toDH(Matrix4d rel_link_frame, Vector3d axis){
        
        // origin_xyz = rel_link_frame[0:3, 3]
        Vector3d temp = new Vector3d();
        rel_link_frame.get(temp);
        Point3d origin_xyz = new Point3d(temp);
        
        // z_axis = np.array([0, 0, 1])
        Vector3d z_axis = new Vector3d(0d,0d,1d);
                
        Chain.DH dh_params;
        
        
        // Collinear case
        
        // The simplest case, take the z-axis (remember the parent frame is 
        // assumed to already be DH parameterized frame) and test if it is 
        // collinear given the origin of the child frame and a vector describing 
        // the joint axis.
        //if gh.are_collinear(np.zeros(3), z_axis, origin_xyz, axis):
        if (LineUtils.areCollinear(new Point3d(), z_axis, origin_xyz, axis)){
            //dh_params = self.processCollinearCase(origin_xyz, rel_link_frame[0:3, 0])
            Matrix3d rel_link_rot = new Matrix3d();
            rel_link_frame.get(rel_link_rot);
            Vector3d x = new Vector3d();
            rel_link_rot.getColumn(0, x);
            dh_params = processCollinearCase(origin_xyz, x);

            
        // Parallel case
        
        } else if (LineUtils.areParallel(z_axis, axis)){
            dh_params = processParallelCase(origin_xyz);

            
        // Intersect case
        
        // elif gh.lines_intersect(np.zeros(3), z_axis, origin_xyz, axis)[0]:
        } else if (LineUtils.linesIntersect(new Point3d(),
                z_axis, origin_xyz, axis) != null){
            dh_params = processIntersectionCase(origin_xyz, axis);

            
        // Skew case
        
        } else {
            // dh_params = self.process_skew_case(origin_xyz, axis)
            dh_params = processSkewCase(origin_xyz, axis);
        }
        return dh_params;
    }
    
    private static Chain.DH processCollinearCase(Point3d origin, Vector3d xaxis){
        //dh_params = np.zeros(4)
        double d = origin.z; // dh_params[0] = origin[2]
        // dh_params[1] = math.atan2(xaxis[1], xaxis[0])
        double theta = Math.atan2(xaxis.y, xaxis.x);
        return new Chain.DH(d, theta, 0d, 0d);
    }
    
    private static Chain.DH processParallelCase(Point3d origin){
        //dh_params = np.zeros(4)
        //dh_params[0] = origin[2]
        double d = origin.z;
        //dh_params[1] = math.atan2(origin[1], origin[0]) 
        double theta = Math.atan2(origin.y,origin.x);
        //dh_params[2] = math.sqrt(origin[0]**2 + origin[1]**2) // r
        double r = Math.sqrt(Math.pow(origin.x,2)+Math.pow(origin.y,2));
        return new Chain.DH(d, theta, r, 0);
    }
    
    private static Chain.DH processIntersectionCase(Point3d origin, Vector3d axis){
        //dh_params = np.zeros(4)
        //dh_params[0] = gh.lines_intersect(np.zeros(3), np.array([0, 0, 1]), origin, axis)[1][0]
        double d = LineUtils.linesIntersect(new Point3d(), new Vector3d(0d,0d,1d),
               origin, axis)[0];

        //zaxis = np.array([0., 0., 1.])
        //xaxis = np.array([1., 0., 0.])
        Vector3d zaxis = new Vector3d(0d,0d,1d); 
        Vector3d xaxis = new Vector3d(1d, 0d, 0d); 

        //for i in range(0,3){
        //    if abs(axis[i]) < 1.e-5:
        //        axis[i] = 0
        //}
        if (Math.abs(axis.x) < 1.e-5) axis.x = 0d;
        if (Math.abs(axis.y) < 1.e-5) axis.y = 0d;
        if (Math.abs(axis.z) < 1.e-5) axis.z = 0d;
        
        Vector3d cn = new Vector3d();
        cn.cross(zaxis, axis);//np.cross(zaxis, axis)
        
        //for i in range(0,3):
        //    if abs(cn[i]) < 1.e-6:
        //        cn[i] = 0
        if (Math.abs(cn.x) < 1.e-6) cn.x = 0d;
        if (Math.abs(cn.y) < 1.e-6) cn.y = 0d;
        if (Math.abs(cn.z) < 1.e-6) cn.z = 0d;
        
        //if (cn[0] < 0):
        //    cn = cn * -1
        if (cn.x < 0) cn.scale(-1);
        
        //dh_params[1] = math.atan2(cn[1], cn[0])
        double theta = Math.atan2(cn.y, cn.x);
        //print(math.atan2(np.dot(np.cross(xaxis, cn), zaxis), np.dot(xaxis, cn)))

        //dh_params[2] = 0
        double r = 0;

        //vn = cn / np.linalg.norm(cn)
        Vector3d vn = new Vector3d(cn);
        vn.normalize();
        //dh_params[3] = math.atan2(np.dot(np.cross(zaxis, axis), vn), np.dot(zaxis, axis))
        Vector3d temp = new Vector3d();
        temp.cross(zaxis, axis);
        double alpha = Math.atan2(temp.dot(vn), zaxis.dot(axis));
        return new Chain.DH(d, theta, r, alpha);
    }
    
    private static Chain.DH processSkewCase(Point3d origin, Vector3d direction){
        // Find closest points along parent z-axis (pointA) and joint axis (pointB)
        //  t = -1.0 * (origin[0] * direction[0] + origin[1] * direction[1]) / (direction[0]**2 + direction[1]**2)
        double t = -1.0 * (origin.x * direction.x + origin.y * direction.y) / (Math.pow(direction.x,2d) + Math.pow(direction.y,2d));
        // pointB = origin + t * direction
        Point3d pointB = new Point3d(origin);
        Vector3d vec = new Vector3d(direction); 
        vec.scale(t);
        pointB.add(vec);
        
        // pointA[2] = pointB[2]
        Point3d pointA = new Point3d();
        pointA.z = pointB.z;

        // 'd' is offset along parent z axis
        // dh_params[0] = pointA[2]
        double d = pointA.z; // dh_params[0] = pointA[2]

        // 'r' is the length of the common normal
        //  dh_params[2] = np.linalg.norm(pointB - pointA)
        Vector3d rVec = new Vector3d(pointB);
        rVec.sub(pointA);
        double r = rVec.length();

        // 'theta' is the angle between the x-axis and the common normal
        // dh_params[1] = math.atan2(pointB[1], pointB[0])
        double theta = Math.atan2(pointB.y, pointB.x); 

        // 'alpha' is the angle between the current z-axis and the joint axis
        // Awesome way to get signed angle:
        // https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane/33920320#33920320
        //cn = pointB - pointA
        Point3d cn = new Point3d(pointB);
        cn.sub(pointA);
        // vn = cn / np.linalg.norm(cn)
        Vector3d vn = new Vector3d(cn);
        vn.normalize();
        // zaxis = np.array([0, 0, 1])
        Vector3d zaxis = new Vector3d(0,0,1d); 
        //dh_params[3] = math.atan2(np.dot(np.cross(zaxis, direction), vn), np.dot(zaxis, direction))
        Vector3d temp = new Vector3d();
        temp.cross(zaxis, direction);
        double alpha = Math.atan2(temp.dot(vn), zaxis.dot(direction));
        return new Chain.DH(d, theta, r, alpha);
    }
}
