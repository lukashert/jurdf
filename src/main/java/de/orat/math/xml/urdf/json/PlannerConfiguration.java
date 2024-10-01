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
public class PlannerConfiguration {
    
    @JsonSerialize(using = ToStringSerializer.class)
    private final int hasValidTorqueModel; //TODO irgendwie nach boolean konvertieren?
    @JsonSerialize(using = ToStringSerializer.class)
    private final double maxBlendRatio; 
    private final ValidCircleRadius validCircleRadius; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double plannerRampTime; 
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public PlannerConfiguration(@JsonProperty("hasValidTorqueModel") int hasValidTorqueModel,
                                @JsonProperty("maxBlendRatio") double maxBlendRatio,
                                @JsonProperty("validCircleRadius") ValidCircleRadius validCircleRadius,
                                @JsonProperty("plannerRampTime") double plannerRampTime){
        this.hasValidTorqueModel = hasValidTorqueModel;
        this.maxBlendRatio = maxBlendRatio;
        this.validCircleRadius = validCircleRadius;
        this.plannerRampTime = plannerRampTime;
    }
}
