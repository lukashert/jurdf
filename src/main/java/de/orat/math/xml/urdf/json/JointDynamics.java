package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.ToStringSerializer;
import lombok.Getter;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
@Getter 
public class JointDynamics {
    
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxVelocity; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxAcceleration; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxDeceleration; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxTorque; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double effTorque; 
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public JointDynamics(@JsonProperty("maxVelocity") double maxVelocity, 
                         @JsonProperty("maxAcceleration") double maxAcceleration,
                         @JsonProperty("maxDeceleration") double maxDeceleration,
                         @JsonProperty("maxTorque") double maxTorque,
                         @JsonProperty("effTorque") double effTorque){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxTorque = maxTorque;
        this.effTorque = effTorque;
    }
}
