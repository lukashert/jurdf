package de.orat.math.xml.urdf.visual;

/**
 * VisualElements can be used for collision detection and visual representation
 * and are used as subelements of link object.
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public abstract class Shape {
    
    public enum ShapeType {box, cylinder, sphere, mesh};
    
    private String name;
    
    public abstract String toXML();
    
    public String getName(){
        return name;
    }
    
    public abstract ShapeType getShapeType();
}
