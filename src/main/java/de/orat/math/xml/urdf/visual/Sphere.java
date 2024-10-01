package de.orat.math.xml.urdf.visual;

/**
 * The origin of the sphere is in its center.
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Sphere extends Shape {
    
    private double radius;
    
    public Sphere(double radius){
        this.radius = radius;
    }

    public double getRadius(){
        return radius;
    }
    
    @Override
    public String toXML() {
        StringBuilder sb = new StringBuilder();
        sb.append("          <sphere radius=\"");
        sb.append(String.valueOf(radius));
        sb.append("\">\n");
        return sb.toString();
    }
    
    public ShapeType getShapeType(){
        return ShapeType.sphere;
    }
}
