package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonRootName;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.ToStringSerializer;
import de.orat.math.xml.urdf.api.Chain.DH;
import lombok.Getter;

/**
 * https://reflectoring.io/jackson/
 * Damit könnte ich getter und setter automatisch erzeugen lassen etc.
 * 
 * deprecated,da ich direkt meine DH-Parameter records verwenden kann
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
@Getter  
public class DHParameter {
    
    // [mm][deg]
    @JsonSerialize(using = ToStringSerializer.class)
    private final double a; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double d; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double alpha; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double theta; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final int reverseRotationDirection; 
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public DHParameter(@JsonProperty("a") double a, 
                       @JsonProperty("d") double d, 
                       @JsonProperty("alpha") double alpha, 
                       @JsonProperty("theta") double theta, 
                       @JsonProperty("reverseRotationDirection") int reverseRotationDirection){
        this.a = a;
        this.d = d;
        this.alpha = alpha;
        this.theta = theta;
        this.reverseRotationDirection = reverseRotationDirection;
    }
    public DH toDH(){
        return new DH(d/1000d, theta, a/1000d, alpha);
    }
}
