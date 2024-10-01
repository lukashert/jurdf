package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.ToStringSerializer;
import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.Chain.RPY;
import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.api.InertialParameters;
import lombok.Getter;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
@Getter 
public class Link {

    // cog [mm]
    @JsonSerialize(using = ToStringSerializer.class)
    private final double mass; 
    private final Inertia inertia; 
    private final CenterOfMass centerOfMass;
    @JsonSerialize(using = ToStringSerializer.class)
    private final double staticFriction; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double viscousFriction; 
    @JsonSerialize(using = ToStringSerializer.class)
    private final double actuatorInertia; 
    
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public Link(@JsonProperty("mass") double mass,
                 @JsonProperty("inertia") Inertia inertia,
                 @JsonProperty("centerOfMass") CenterOfMass centerOfMass,
                 @JsonProperty("staticFriction") double staticFriction,
                 @JsonProperty("viscousFriction") double viscousFriction,
                 @JsonProperty("actuatorInertia") double actuatorInertia){
        this.mass = mass;
        this.inertia = inertia;
        this.centerOfMass = centerOfMass;
        this.staticFriction = staticFriction;
        this.viscousFriction = viscousFriction;
        this.actuatorInertia = actuatorInertia;
    }
    
    
    public de.orat.math.xml.urdf.api.InertialParameters getInertialParameters(){
        Chain.RPYXYZ rpyxyz = new RPYXYZ(new RPY(0d,0d,0d), centerOfMass.getPosition());
        // TODO woher bekomme ich rpy, vermutlich aus dem vorherigen joint
        // möglicherweise muss ich centerOfMass vorher noch entsprechend RPY drehen
        
        InertialParameters.InertialMatrixElements imm = inertia.toInertialMatrixElements();
        return new de.orat.math.xml.urdf.api.InertialParameters(rpyxyz, mass, imm);
    }
    
    public de.orat.math.xml.urdf.api.Link toURDFLink(String linkName){
        de.orat.math.xml.urdf.api.Link urdfLink = new de.orat.math.xml.urdf.api.Link(linkName);
        urdfLink.set(getInertialParameters());
        //TODO
        // set friction parameters und actuator inertia müssen vermutlich in joint
        // bzw. in actuator-fake-links gespeichert werden
        return urdfLink;
    }
}
