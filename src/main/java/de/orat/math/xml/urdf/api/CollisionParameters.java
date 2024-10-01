package de.orat.math.xml.urdf.api;

import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.visual.Shape;

/**
 * The collision properties of a link. Note that this can be different from the 
 * visual properties of a link, for example, simpler collision models are often 
 * used to reduce computation time. Note: multiple instances of <collision> tags 
 * can exist for the same link. The union of the geometry they define forms the 
 * collision representation of the link.<p>
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class CollisionParameters {
    
    private final RPYXYZ rpyxyz;
    private final Shape shape;
    private String name;
    
    public CollisionParameters(RPYXYZ rpyxyz, Shape shape){
        this.rpyxyz = rpyxyz;
        this.shape = shape;
    }
    public void setName(String name){
        this.name = name;
    }
    public String getName(){
        return name;
    }
    
    public RPYXYZ getRPYXYZ(){
        return rpyxyz;
    }
    
    public Shape getShape(){
        return shape;
    }
    
    public String toXML(){
        StringBuilder sb = new StringBuilder();
        sb.append("      <collsion");
        if (name != null){
            sb.append(" name=\"");
            sb.append(name);
            sb.append("\">\n");
        } else {
            sb.append(">\n");
        }
        sb.append("        ");
        sb.append(Chain.toXML(rpyxyz));
        sb.append("        <geometry>\n");
        sb.append(shape.toXML());
        sb.append("        </geometry>\n");
        sb.append("      </visual>\n");
        return sb.toString();
    }
}
