package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.ToStringSerializer;
import lombok.Getter;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
@Getter  
public class CenterOfMass {
    
    @JsonSerialize(using = ToStringSerializer.class)
    private final double x;
    @JsonSerialize(using = ToStringSerializer.class)
    private final double y;
    @JsonSerialize(using = ToStringSerializer.class)
    private final double z;
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public CenterOfMass(@JsonProperty("x") double x, 
                        @JsonProperty("y") double y, 
                        @JsonProperty("z") double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    Vector3d getPosition(){
        return new Vector3d(x/1000d,y/1000d,z/1000d);
    }
}
