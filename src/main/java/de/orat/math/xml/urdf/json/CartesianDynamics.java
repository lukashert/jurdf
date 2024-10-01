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
public class CartesianDynamics {
    
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxVelocity; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxAcceleration; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxAngularVelocity; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxAngularAcceleration; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxElbowVelocity; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxElbowAcceleration; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxElbowForce; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxTcpForce; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double applyAccelerationLimitToComponents; 
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public CartesianDynamics(@JsonProperty("maxVelocity") double maxVelocity, 
                             @JsonProperty("maxAcceleration") double maxAcceleration,
                             @JsonProperty("maxAngularVelocity") double maxAngularVelocity,
                             @JsonProperty("maxAngularAcceleration") double maxAngularAcceleration,
                             @JsonProperty("maxElbowVelocity") double maxElbowVelocity,
                             @JsonProperty("maxElbowAcceleration") double maxElbowAcceleration,
                             @JsonProperty("maxElbowForce") double maxElbowForce,
                             @JsonProperty("maxTcpForce") double maxTcpForce,
                             @JsonProperty("applyAccelerationLimitToComponents") double applyAccelerationLimitToComponents){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
        this.maxElbowVelocity = maxElbowVelocity;
        this.maxElbowAcceleration = maxElbowAcceleration;
        this.maxElbowForce = maxElbowForce;
        this.maxTcpForce = maxTcpForce;
        this.applyAccelerationLimitToComponents = applyAccelerationLimitToComponents;
    }
}
