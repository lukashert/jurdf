package de.orat.math.xml.urdf.api;

import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.visual.Material;
import de.orat.math.xml.urdf.visual.Shape;
import java.awt.Color;

/**
 * This element specifies the shape of the object (box, cylinder, etc.) for 
 * visualization purposes. Note: multiple instances of <visual> tags can exist 
 * for the same link. The union of the geometry they define forms the visual 
 * representation of the link. 
 * 
 * TODO
 * - add material
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class VisualParameters {
    
    private String name; // optional
    private Chain.RPYXYZ rpyxyz;
    private Shape shape;
    private Material material;
    
    public VisualParameters(RPYXYZ rpyxyz, Shape ele){
        this.rpyxyz = rpyxyz;
        this.shape = ele;
    }
    public void set(Material material){
        this.material = material;
    }
    /**
     * Specifies a name for a part of a link's geometry. This is useful to be 
     * able to refer to specific bits of the geometry of a link. 
     * 
     * @param name 
     */
    public void setName(String name){
        this.name = name;
    }
    public String getName(){
        return name;
    }
    
    /**
     * Get the color of the shape.
     * 
     * @return Color.Gray if material or the color of the material is not set.
     */
    public Color getColor(){
        Color color = Color.GRAY;
        if (material != null){
            Color col = material.getColor();
            if (col != null){
                color = col;
            }
        }
        return color;
    }
    public RPYXYZ getRPYXYZ(){
        return rpyxyz;
    }
    public Material getMaterial(){
        return material;
    }
    public Shape getShape(){
        return shape;
    }
    
    public String toXML(){
        StringBuilder sb = new StringBuilder();
        sb.append("      <visual");
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
     
    /*outstring = outstring + "\t\t<visual>\n"
            outstring = outstring + "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3])
            outstring = outstring + "\t\t\t<geometry>\n"
            outstring = outstring + "\t\t\t\t<cylinder length='1' radius='0.5'/>\n"
            outstring = outstring + "\t\t\t</geometry>\n"
            outstring = outstring + "\t\t\t<material name='blue'/>\n"
            outstring = outstring + "\t\t</visual>\n"
            ou

    */
}
