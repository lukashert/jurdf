package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.List;
import lombok.Getter;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
@Getter 
public class Dynamics {
    
    private final CartesianDynamics cartesianDynamics;
    private final List<JointDynamics> jointDynamics;
    private final String mode; //TODO Umstellung auf eine Enumeration
    // "mode": "OPERATION_AUTO"
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public Dynamics(@JsonProperty("cartesianDynamics") CartesianDynamics cartesianDynamics,
                    @JsonProperty("jointDynamics") List<JointDynamics> jointDynamics,
                    @JsonProperty("mode") String mode){
        this.cartesianDynamics = cartesianDynamics;
        this.jointDynamics = jointDynamics;
        this.mode = mode;
    }
}
