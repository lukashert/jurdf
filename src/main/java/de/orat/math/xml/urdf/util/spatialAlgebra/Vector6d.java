package de.orat.math.xml.urdf.util.spatialAlgebra;

import org.jogamp.vecmath.GMatrix;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Vector6d extends GMatrix {
    
    public Vector6d() {
        super(6, 1);
    }
    
    public double getElement(int row){
        return getElement(row, 0);
    }
}
