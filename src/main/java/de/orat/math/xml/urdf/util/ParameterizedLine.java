package de.orat.math.xml.urdf.util;

import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Vector3d;

/**
 * A line defined by a startpoint and a direction.
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class ParameterizedLine {
    
    public static double EPSILON = 1.0e-5;
    
    public record IntersectionParameters(double t, Point3d p){};
    
    private Point3d origin;
    private Vector3d dir;

    public ParameterizedLine(Point3d p0, Point3d p1){
        origin = new Point3d(p0);
        dir = new Vector3d(p1);
        dir.sub(p0);
    }
    
    public Vector3d direction(){
        return new Vector3d(dir);
    }
    
    /**
     * The projection of a point p onto the line.
     * 
     */
    public Point3d projection(Point3d p){
        throw new RuntimeException("not yet implemented and not needed!");
    }
    /**
     * The parameter value of the intersection between this and the given plane.
     * 
     * l(t) = origin + t*dir
     * 
     * @param plane
     * @return 
     */
    public double intersectionParameter(Plane4d plane){
        Point3d p = plane.getPoint(); // point auf der Ebene
        
        double dot = (new Vector3d(p)).dot(dir);
        if (Math.abs(dot) > EPSILON){
            // The factor of the point between p0 -> p1 (0 - 1)
            // if 'fac' is between (0 - 1) the point intersects with the segment.
            // Otherwise:
            //  < 0.0: behind p0.
            //  > 1.0: infront of p1.
            
            // w = sub_v3v3(p0, p_co)
            Vector3d w = new Vector3d(origin);
            w.sub(p);
            
            //fac = -dot_v3v3(p_no, w) / dot
            return -plane.getN().dot(w)/dot;
        }
        return Double.NaN;
    }
    
    public Point3d intersectioPoint(Plane4d plane){
        double fac = intersectionParameter(plane);
        if (!Double.isNaN(fac)){
            return intersectionPoint(fac);
        }
        return null;
    }
    
    private Point3d intersectionPoint(double fac){
        // u = mul_v3_fl(u, fac)
        Vector3d u = new Vector3d(dir);
        u.scale(fac);

        // return add_v3v3(p0, u)
        Point3d result = new Point3d(origin);
        result.add(u);
        return result;
    }
    
    /**
     * The point of intersection between this and the given plane.
     * 
     * https://stackoverflow.com/questions/5666222/3d-line-plane-intersection
     * 
     * @param plane
     * @return null, if the line is parallel to the plane, else return the 
     * intersection point and the intersection parameter.
     */
    public IntersectionParameters intersectionParameters(Plane4d plane){
        double fac = intersectionParameter(plane);
        if (!Double.isNaN(fac)){
            return new IntersectionParameters(fac, intersectionPoint(fac)); 
        }
        
        // The segment is parallel to plane.
        return null;
    }
}
