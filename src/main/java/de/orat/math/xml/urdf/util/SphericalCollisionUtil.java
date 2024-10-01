package de.orat.math.xml.urdf.util;

import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.CollisionParameters;
import de.orat.math.xml.urdf.visual.Sphere;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class SphericalCollisionUtil {
    
    private Chain chain;
    
    public SphericalCollisionUtil(Chain chain){
        this.chain = chain;
    }
    
    private static CollisionParameters createSphericalCollisionParameters(Vector3d p, double radius) {
        return new CollisionParameters(new Chain.RPYXYZ(new Chain.RPY(0,0,0), p), new Sphere(radius));
    }
    
    public void addSphere(String linkName, Vector3d p, double radius){
        chain.getLink(linkName).add(createSphericalCollisionParameters(p, radius));
    }
}
