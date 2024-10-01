package de.orat.math.xml.urdf.api;

import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class InertialParameters {
    
   public record InertialMatrixElements(double ixx, double ixy, double ixz, double iyy, double iyz, double izz){};
   
   private final RPYXYZ rpyxyz;
   private final double mass;
   private final InertialMatrixElements imm;
   
   public InertialParameters(RPYXYZ rpyxyz, double mass, InertialMatrixElements imm){
       this.imm = imm;
       this.mass = mass;
       this.rpyxyz = rpyxyz;
   }
   
   /*
   <xacro:macro name="cylinder_inertial" params="radius length mass *origin">
    <inertial>
      <mass value="${mass}" />
      <xacro:insert_block name="origin" />
      <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
        iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
        izz="${0.5 * mass * radius * radius}" />
    </inertial>
  </xacro:macro>
   */
   // solid cylinder in direction of z and the midpoint of the cylinder as origin
   // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
   public static InertialMatrixElements createIntertialMatrixElements4Cylinder(
                  double radius, double length, double mass, Vector3d origin){
        return new InertialMatrixElements(1d/12d * mass * (3 * radius * radius + length * length), 
                0d, 0d, 1d/12d * mass * (3 * radius * radius + length * length),
                0d, 0.5 * mass * radius * radius);
   }
   
   // https://github.com/gstavrinos/calc-inertia/blob/master/calc_inertia_for_urdf.py
   // da findet sich python code für inertia for cylinder, box, etc.
   
   public RPYXYZ getRPYXYZ(){
        return rpyxyz;
   }
    
   public String toXML(){
       StringBuilder sb = new StringBuilder();
       if (imm != null){
        sb.append("      <inertial>\n        ");
        if (rpyxyz !=null){
            sb.append(Chain.toXML(rpyxyz));
        }
        if (!Double.isNaN(mass)){
            sb.append("<mass value=\"");
            sb.append(String.valueOf(mass));
            sb.append("\">\n");
        }
        if (imm != null){
            sb.append("        <inertia ixx=\"");
            sb.append(String.valueOf(imm.ixx));
            sb.append("\" ixy=\"");
            sb.append(String.valueOf(imm.ixy));
            sb.append("\" ixz=\"");
            sb.append(String.valueOf(imm.ixz));
            sb.append("\" iyy=\"");
            sb.append(String.valueOf(imm.iyy));
            sb.append("\" iyz=\"");
            sb.append(String.valueOf(imm.iyz));
            sb.append("\" izz=\"");
            sb.append(String.valueOf(imm.izz));
            sb.append("\">\n");
        }
        sb.append("      </inertial>\n");
       }
       return sb.toString();
   }
   
   /*
    <inertial>
   3      <origin xyz="0 0 0.5" rpy="0 0 0"/>
   4      <mass value="1"/>
   5      <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
   6    </inertial>
   */
}
