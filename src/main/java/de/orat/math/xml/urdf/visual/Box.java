package de.orat.math.xml.urdf.visual;

/**
 * The origin of the box is in its center. 
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Box extends Shape {

    // a Size attribut contains the three side liengths of a box. 
    public record Size(double x, double y, double z){};
    
    private Size size;
    
    public Box(Size size){
        this.size = size;
    }
    
    public Size getSize(){
        return size;
    }
    @Override
    public String toXML() {
        StringBuilder sb = new StringBuilder();
        sb.append("          <box size=\"");
        sb.append(String.valueOf(size.x));
        sb.append(" ");
        sb.append(String.valueOf(size.y));
        sb.append(" ");
        sb.append(String.valueOf(size.z));
        sb.append("\">\n");
        return sb.toString();
    }
    
    public ShapeType getShapeType(){
        return ShapeType.box;
    }
}
