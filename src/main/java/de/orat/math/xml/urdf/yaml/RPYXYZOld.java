package de.orat.math.xml.urdf.yaml;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.ToStringSerializer;
import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.Chain.RPY;
import lombok.AllArgsConstructor;
import lombok.NoArgsConstructor;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class RPYXYZOld {
    
    //@JsonProperty("x")
    @JsonSerialize(using = ToStringSerializer.class)
    private double x;
    @JsonSerialize(using = ToStringSerializer.class)
    private double y;
    @JsonSerialize(using = ToStringSerializer.class)
    private double z;
    @JsonSerialize(using = ToStringSerializer.class)
    private double roll;
    @JsonSerialize(using = ToStringSerializer.class)
    private double pitch;
    @JsonSerialize(using = ToStringSerializer.class)
    private double yaw;
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public RPYXYZOld(@JsonProperty("x") String x, 
                       @JsonProperty("y") String y, 
                       @JsonProperty("z") String z, 
                       @JsonProperty("roll") String roll, 
                       @JsonProperty("pitch") String pitch,
                       @JsonProperty("yaw") String yaw){
        this.x = Double.parseDouble(x);
        this.y = Double.parseDouble(y);
        this.z = Double.parseDouble(z);
        this.roll = Double.parseDouble(roll);
        this.pitch = Double.parseDouble(pitch);
        this.yaw = Double.parseDouble(yaw);
    }
    /*public RPYXYZOld(@JsonProperty("x") double x, 
                       @JsonProperty("y") double y, 
                       @JsonProperty("z") double z, 
                       @JsonProperty("roll") double roll, 
                       @JsonProperty("pitch") double pitch,
                       @JsonProperty("yaw") double yaw){
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }*/
    public Chain.RPYXYZ getRPYXYZ(){
        return new Chain.RPYXYZ(new RPY(roll, pitch, yaw), new Vector3d(x,y,z));
    }
}
