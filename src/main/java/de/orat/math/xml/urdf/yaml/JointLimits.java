package de.orat.math.xml.urdf.yaml;

import com.fasterxml.jackson.annotation.JsonAnySetter;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.ToStringSerializer;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import lombok.AllArgsConstructor;
import lombok.Getter;

/**
 *
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 * 
 */
public class JointLimits {
    
    @JsonProperty("joint_limits")
    public Map<String, JointLimit> jointLimits = new LinkedHashMap<>();
    
    public JointLimits(){}
    
    
    @JsonAnySetter
    public void setJointLimit(String key, JointLimit jointLimit) {
        jointLimits.put(key, jointLimit);
    }
    
    public JointLimit get(String key){
        return jointLimits.get(key);
    }
    public List<String> names(){
        return new ArrayList<>(jointLimits.keySet());
    }
    
    @Getter  
    public static class JointLimit {
        
        public JointLimit(){}
        
        @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public JointLimit(@JsonProperty("has_acceleration_limits") boolean has_acceleration_limits, 
                       @JsonProperty("has_effort_limits") boolean has_effort_limits, 
                       @JsonProperty("has_position_limits") boolean has_position_limits, 
                       @JsonProperty("has_velocity_limits") boolean has_velocity_limits, 
                       
                       @JsonProperty("max_effort") double max_effort,
                       @JsonProperty("max_position") double max_position,
                       @JsonProperty("max_velocity") double max_velocity,
                       @JsonProperty("min_position") double min_position){
            
            this.has_acceleration_limits = has_acceleration_limits;
            this.has_effort_limits = has_effort_limits;
            this.has_position_limits = has_position_limits;
            this.has_velocity_limits = has_velocity_limits;
            this.max_effort = max_effort;
            //this.max_position = max_position;
            //this.max_velocity = max_velocity;
            //this.min_position = min_position;
        }
        
        @JsonProperty("has_acceleration_limits")
        private boolean has_acceleration_limits; //: false
        @JsonProperty("has_effort_limits")
        private boolean has_effort_limits; //: true
        @JsonProperty("has_position_limits")
        private boolean has_position_limits; //: true
        @JsonProperty("has_velocity_limits")
        private boolean has_velocity_limits; //: true
        @JsonProperty("max_effort")
        private double max_effort; //: 150.0
        
        //max_position: !degrees  180.0
        //max_velocity: !degrees  180.0
        //min_position: !degrees -180.0
        
    }
}
