package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.ToStringSerializer;
import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.InertialParameters;
import de.orat.math.xml.urdf.api.InertialParameters.InertialMatrixElements;
import lombok.Getter;

/**
 *
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
@Getter 
public class Inertia {
    
    @JsonSerialize(using = ToStringSerializer.class)
    private final double xx; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double yy; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double zz; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double xy; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double xz; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double yz; 
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public Inertia(@JsonProperty("xx") double xx, @JsonProperty("yy") double yy, 
                   @JsonProperty("zz") double zz, @JsonProperty("xy") double xy, 
                   @JsonProperty("xz") double xz, @JsonProperty("yz") double yz){
        this.xx = xx;
        this.yy = yy;
        this.zz = zz;
        this.xy = xy;
        this.xz = xz;
        this.yz = yz;
    }
    
    public InertialParameters.InertialMatrixElements toInertialMatrixElements(){
        return new InertialMatrixElements(xx, xy, xz, yy, yz, zz);
    }
}
