package de.orat.math.xml.urdf.util;

import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class LineUtils {

    public static double EPSILON = 1.0e-5;

    /**
     * Determine if two vectors are parallel.
     *
     * https://github.com/mcevoyandy/urdf_to_dh/blob/main/urdf_to_dh/geometry_helpers.py
     * 
     * @param vec1
     * @param vec2
     * @return 
     */
    public static boolean areParallel(Vector3d vec1, Vector3d vec2){
        Vector3d vec1_unit = new Vector3d(vec1);
        vec1_unit.normalize();
        Vector3d vec2_unit = new Vector3d(vec2);
        vec2_unit.normalize();

        Vector3d cross = new Vector3d();
        cross.cross(vec1, vec2);
        if (cross.x > EPSILON) return false;
        if (cross.y > EPSILON) return false;
        return cross.z <= EPSILON;
    }
    
    // https://numpy.org/doc/stable/reference/generated/numpy.isclose.html
    /**
     * The relative difference (rtol * abs(b)) and the absolute difference atol 
     * are added together to compare against the absolute difference between a and b.
     * 
     * @param a
     * @param b
     * @return 
     */
    public static boolean areClose(Point3d a, Point3d b){
        double rtol=1e-05; // The relative tolerance parameter
        double atol=1e-08; // The absolute tolerance parameter
        double absDistance = a.distanceL1(b);
        Vector3d bv = new Vector3d(b);
        return rtol*bv.length()+atol > absDistance;
    }
    public static boolean areClose(double a, double b){
        double atol=1e-08;
        return Math.abs(a-b) < atol;
    }
    
    /**
     * Determine if two lines are collinear.
     * 
     * Points are collinear if lie on the same line. So two points are always 
     * collinear. Two lines are collinear if they are identical.
     * 
     * https://github.com/mcevoyandy/urdf_to_dh/blob/main/urdf_to_dh/geometry_helpers.py
     */
    public static boolean areCollinear(Point3d point1, Vector3d vec1, Point3d point2, Vector3d vec2){
    
        // To be collinear, vectors must be parallel
        if (!areParallel(vec1, vec2)) return false;

        // If parallel and point1 is coincident with point2, vectors are collinear
        if (areClose(point1, point2)) return true;

        // If vectors are parallel, point2 can be defined as p2 = p1 + t * v1
        //t = np.zeros(3)
        //for idx in range(0, 3){
        //    if vec1[idx] != 0:
        //        t[idx] = (point2[idx] - point1[idx]) / vec1[idx]
        //}
        Vector3d t = new Vector3d();
        if (vec1.x != 0d) t.x = (point2.x-point1.x)/vec1.x;
        if (vec1.y != 0d) t.y = (point2.y-point1.y)/vec1.y;
        if (vec1.z != 0d) t.z = (point2.z-point1.z)/vec1.z;
        //p2 = point1 + t * vec1
        Point3d p2 = new Point3d(point1);
        p2.add(new Vector3d(t.x*vec1.x, t.y*vec1.y, t.z*vec1.z));

        //return np.allclose(p2, point2)
        return areClose(p2, point2);
    }

    /**
     * Determine if two lines intersect.
     *
     * https://github.com/mcevoyandy/urdf_to_dh/blob/main/urdf_to_dh/geometry_helpers.py
     * 
     * @param point1v a point on the first line
     * @param vec1 direction vector of the first line
     * @param point2v a point on the second line
     * @param vec2 direction vector of the second line
     * @return double[2] = d,? else null
     */
    public static double[] linesIntersect(Point3d point1v, Vector3d vec1v, 
                                          Point3d point2v, Vector3d vec2v){
        return null;
    }
    
    // original implementation aber unvollständig
    public static double[] linesIntersectOrig(Point3d point1v, Vector3d vec1v, 
                                          Point3d point2v, Vector3d vec2v){
        double epsilon = EPSILON/10d; // 1e-6;
        //x = np.zeros(2)
        double[] x = new double[2];
        
        if (areCollinear(point1v, vec1v, point2v, vec2v)) return null;

        // If lines are parallel, lines don't intersect
        if (areParallel(vec1v, vec2v)) return null;

        double[] vec1 = new double[]{vec1v.x, vec1v.y, vec1v.z};
        double[] vec2 = new double[]{vec2v.x, vec2v.y, vec2v.z};
        double[] point1 = new double[]{point1v.x, point1v.y, point1v.z};
        double[] point2 = new double[]{point2v.x, point2v.y, point2v.z};
        // Test if lines intersect. Need to find non-singular pair to solve for coefficients
        //for idx in range(0,3):
        for (int idx=0;idx<3;idx++){
            int i = idx;
            int j = (idx + 1) % 3; 
            //A = np.array([[vec1[i], -vec2[i]], [vec1[j], -vec2[j]]])
            double[][] A = new double[][]{{vec1[i], -vec2[i]},{vec1[j], -vec2[j]}};
            //b = np.array([[point2v[i] - point1v[i]], [point2v[j] - point1v[j]]])
            double[] b = new double[]{point2[i]-point1[i], point2[j]-point1[j]};
            
            // If singular matrix, go to next set
            // if (np.isclose(np.linalg.det(A), 0)){
            if (areClose(det2x2(A), 0)){
                continue;
            } else {
                //x = np.linalg.solve(A, b)
                //TODO
                x = solve(A, b);
                
                //# Test if solution generates a point of intersection
                //Point3d p1 = point1v + x[0] * vec1
                Point3d p1 = new Point3d(point1v);
                Vector3d v1 = new Vector3d(vec1);
                v1.scale(x[0]);
                p1.add(v1);
                //Point3d p2 = point2v + x[1] * vec2

                //TODO
                //if (all(np.less(np.abs(p1 - p2), epsilon * np.ones(3)))) return /*True,*/ x;
            }
            return null;
        }
        //TODO
        throw new RuntimeException("what to do here?");
    }
    
    private static double det2x2(double[][] m){
        if (m.length != 2 || m[0].length != 2) throw new IllegalArgumentException("Matrix dimensions do not match double[2][2]!");
        return m[0][0]*m[1][1]-m[1][0]*m[0][1];
    }
    private static double[] solve(double[][] A, double[] b){
        Matrix4d test;
        
        throw new RuntimeException("not yet implemented!");
    }
}
