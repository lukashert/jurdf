package de.orat.math.xml.urdf.visual;

/**
 *
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Cylinder extends Shape {

    private double length;
    private double radius;
    
    public Cylinder(double length, double radius){
        this.length = length;
        this.radius = radius;
    }
    public double getLength(){
        return length;
    }
    public double getRadius(){
        return radius;
    }
    @Override
    public String toXML() {
        StringBuilder sb = new StringBuilder();
        sb.append("          <cylinder length=\"");
        sb.append(String.valueOf(length));
        sb.append("\" radius=\"");
        sb.append(String.valueOf(radius));
        sb.append("\">\n");
        return sb.toString();
    }
    
    public ShapeType getShapeType(){
        return ShapeType.cylinder;
    }
}
