package de.orat.math.xml.urdf.yaml;

import com.fasterxml.jackson.annotation.JsonAnySetter;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import lombok.Getter;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Kinematics {
   
    @JsonProperty("kinematics")
    public Map<String, Rpyxyz> kinematics = new LinkedHashMap<>();
    
    //@JsonProperty("hash")
    //public String hash;
    
    public Kinematics(){}
    
    @JsonCreator
    public Kinematics(@JsonProperty("kinematics") Map<String, Rpyxyz> kinematics){
        this.kinematics = kinematics;
    }
    
    @JsonAnySetter
    public void setRpyxyz(String key, Rpyxyz rpyxyz) {
        kinematics.put(key, rpyxyz);
    }
    
    //@JsonAnySetter
    //public void setHash(String hash) {
    //    this.hash = hash;
    //}
    
    public Rpyxyz get(String key){
        return kinematics.get(key);
    }
    public List<String> names(){
        return new ArrayList<>(kinematics.keySet());
    }
    
    @Getter  
    //@AllArgsConstructor
    public static class Rpyxyz {
        
        public Rpyxyz(){}
        
        @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public Rpyxyz(@JsonProperty("x") double x, 
                       @JsonProperty("y") double y, 
                       @JsonProperty("z") double z, 
                       @JsonProperty("roll") double roll, 
                       @JsonProperty("pitch") double pitch,
                       @JsonProperty("yaw") double yaw){
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }
        
        /*@JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public Rpyxyz(@JsonProperty("x") String x, 
                       @JsonProperty("y") String y, 
                       @JsonProperty("z") String z, 
                       @JsonProperty("roll") String roll, 
                       @JsonProperty("pitch") String pitch,
                       @JsonProperty("yaw") String yaw){
            this.x = Double.parseDouble(x);
            this.y = Double.parseDouble(y);
            this.z = Double.parseDouble(z);
            this.roll = Double.parseDouble(roll);
            this.pitch = Double.parseDouble(pitch);
            this.yaw = Double.parseDouble(yaw);
        }*/
        
        
        @JsonProperty("x")
        public double x;
        @JsonProperty("y")
        public double y;
        @JsonProperty("z")
        public double z;
        @JsonProperty("roll")
        public double roll;
        @JsonProperty("pitch")
        public double pitch;
        @JsonProperty("yaw")
        public double yaw;
        
        
        /* damit werden Strings der values in yaml erzeugt, d.h. die Werte werden
           in Anführungszeichen gesetzt
        
        @JsonSerialize(using = ToStringSerializer.class)
        public double x;
        @JsonSerialize(using = ToStringSerializer.class)
        public double y;
        @JsonSerialize(using = ToStringSerializer.class)
        public double z;
        @JsonSerialize(using = ToStringSerializer.class)
        public double roll;
        @JsonSerialize(using = ToStringSerializer.class)
        public double pitch;
        @JsonSerialize(using = ToStringSerializer.class)
        public double yaw;
        */
        
        /*@JsonProperty("x")
        public String x;
        @JsonProperty("y")
        public String y;
        @JsonProperty("z")
        public String z;
        @JsonProperty("roll")
        public String roll;
        @JsonProperty("pitch")
        public String pitch;
        @JsonProperty("yaw")
        public String yaw;*/
    }
}