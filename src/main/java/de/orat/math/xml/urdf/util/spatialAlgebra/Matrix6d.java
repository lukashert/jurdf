package de.orat.math.xml.urdf.util.spatialAlgebra;

import org.jogamp.vecmath.GMatrix;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Matrix6d extends GMatrix {
    
    public Matrix6d(Matrix6d m){
        super(m);
    }
    
    public Matrix6d(){
        super(6,6);
    }
    
    public final void set(int row, int col, Matrix3d m){
        setElement(row, col, m.m00);
        setElement(row, col+1, m.m01);
        setElement(row, col+2, m.m02);
        setElement(row+1, col, m.m10);
        setElement(row+1, col+1, m.m11);
        setElement(row+1, col+2, m.m12);
        setElement(row+2, col, m.m20);
        setElement(row+2, col+1, m.m21);
        setElement(row+2, col+2, m.m22);
    }
    
}
