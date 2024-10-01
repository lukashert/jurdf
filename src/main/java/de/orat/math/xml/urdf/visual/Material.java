package de.orat.math.xml.urdf.visual;

import java.awt.Color;

/**
 * The material of the visual element. It is allowed to specify a material element 
 * outside of the 'link' object, in the top level 'robot' element. From within a 
 * link element you can then reference the material by name.<p>
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Material {
    
    private String colorName;
    private Color color;
    
    public Material(String colorName){
        this.colorName = colorName;
    }
    public void setColor(Color color){
        this.color = color;
    }
    public Color getColor(){
        return color;
    }
    String toXML(){
        StringBuilder sb = new StringBuilder();
        sb.append("<material name=\"");
        sb.append(colorName);
        sb.append("\">\n");
        sb.append("\t<color rgba=\"");
        sb.append(color.getRed());
        sb.append(" ");
        sb.append(color.getGreen());
        sb.append(" ");
        sb.append(color.getBlue());
        sb.append(" ");
        sb.append(color.getAlpha());
        sb.append("\"/>\n");
        sb.append("</material>\n");
        return sb.toString();
    }
    
    /*<material name="Cyan">
          <color rgba="0 1.0 1.0 1.0"/>
      </material>
*/
}
