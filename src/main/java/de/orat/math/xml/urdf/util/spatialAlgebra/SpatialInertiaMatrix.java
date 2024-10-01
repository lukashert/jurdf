package de.orat.math.xml.urdf.util.spatialAlgebra;

import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class SpatialInertiaMatrix extends Matrix6d {
    
    /**
     * def spatial_inertia_matrix_IO(ixx, ixy, ixz, iyy, iyz, izz, mass, c):
    """Returns the 6x6 spatial inertia matrix expressed at the origin."""
    IO = np.zeros([6, 6])
    cx = numpy_skew_symmetric(c)
    inertia_matrix = np.array([[ixx, ixy, ixz],
                               [ixy, iyy, iyz],
                               [ixz, iyz, izz]])

    IO[:3, :3] = inertia_matrix + mass*(np.dot(cx, np.transpose(cx)))
    IO[:3, 3:] = mass*cx
    IO[3:, :3] = mass*np.transpose(cx)

    IO[3, 3] = mass
    IO[4, 4] = mass
    IO[5, 5] = mass

    return IO
     */
    // https://drake.mit.edu/doxygen_cxx/group__multibody__spatial__inertia.html
    /**
     * Returns the 6x6 spatial inertia matrix expressed at the origin.
     * 
     * @param position vector from P to B's center of mass, expressin in F
     * @return 
     * 
     * TODO
     * verschieben der methode als Konstruktor einer eigenen class, damit ich als
     * member methods z.B. getCoM(), getMass() hinzufügen kann
     */
    public SpatialInertiaMatrix(double ixx, double ixy, double ixz, 
            double iyy, double iyz, double izz, double mass, Vector3d c){
        //Matrix6d result = new Matrix6d();
        Matrix3d cx = new SkewMatrix3d(c);
        Matrix3d inertia = new Matrix3d(ixx, ixy, ixz,
                               ixy, iyy, iyz,
                               ixz, iyz, izz);
        // result[:3, :3] = inertia_matrix + mass*(np.dot(cx, np.transpose(cx)))
        Matrix3d cxT = new Matrix3d();
        cxT.transpose(cx);
        Matrix3d m = new Matrix3d();
        m.mul(cx, cxT);
        m.mul(mass);
        m.add(inertia);
        set(0,0,m);
        
        //result[:3, 3:] = mass*cx
        set(0,3,cx);
        
        // in doku ist hier noch ein Vorzeichen, aber in der impl nicht
        //FIXME
        //result[3:, :3] = mass*np.transpose(cx)
        cx.mul(mass);
        set(3,0, cx);
        
        setElement(3, 3, mass);
        setElement(4, 4, mass);
        setElement(5, 5, mass);
    }
    
    public double getMass(){
        return getElement(3, 3);
    }
    public Vector3d getCoM(){
        return new Vector3d(getElement(2,1), 
                       getElement(0,2), getElement(1,0));
    }
}
