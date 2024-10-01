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
public class SoftJointLimits {
    
    @JsonSerialize(using = ToStringSerializer.class)
    private final double lower;
    @JsonSerialize(using = ToStringSerializer.class)
    private final double upper;
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public SoftJointLimits(@JsonProperty("lower") double lower,
                           @JsonProperty("upper") double upper ){
        this.lower = lower;
        this.upper = upper;
    }
}
