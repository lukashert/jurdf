package de.orat.math.xml.urdf.util.spatialAlgebra;

import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Vector3d;

/**
 * A skew symmetric matrix.
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class SkewMatrix3d extends Matrix3d {
    
    /**
     * def numpy_skew_symmetric(v):
    """Returns a skew symmetric matrix from vector."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

     */
    public SkewMatrix3d(Vector3d c){
        super(0, -c.z, c.y, c.z, 0, -c.x, -c.y, c.x, 0);
    }
}
