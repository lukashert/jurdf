package de.orat.math.xml.urdf.yaml;

import com.fasterxml.jackson.annotation.JsonAnySetter;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import java.io.IOException;
import java.io.InputStream;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class PhysicalParameters {
    
    // INDENT_OUTPUT sorgt für Zeilenumbrüche
    private static ObjectMapper objectMapper = (new ObjectMapper(new YAMLFactory()))/*.enable(SerializationFeature.INDENT_OUTPUT)*/;
   
    @JsonProperty("dh_parameters")
    private DHParameters dh_parameters;
    @JsonProperty("offsets")
    private Offsets offsets;
    @JsonProperty("inertia_parameters")
    private InertialParameters inertia_parameters;
    
    public PhysicalParameters(){}
   
    public static PhysicalParameters load(String yamlFileName) throws IOException{
        InputStream inputStream = PhysicalParameters.class.getResourceAsStream(yamlFileName);
        PhysicalParameters physicalParameters = objectMapper.readValue(inputStream, PhysicalParameters.class);
        return physicalParameters;
    }
    
    public static void save(String yamlFileName, PhysicalParameters physicalParameters) throws JsonProcessingException{
        String parsedJson = objectMapper.writeValueAsString(physicalParameters);
        //TODO in Datei schreiben
        System.out.println(parsedJson);
    }
    
    public class DHParameters {
        
        @JsonProperty("d1")
        double d1;//: 0.163
        @JsonProperty("a2")
        double a2; //: -0.42500
        @JsonProperty("a3")
        double a3; //: -0.39225
        @JsonProperty("d4")
        double d4; //: 0.134         # wrist1_length = d4 - elbow_offset - shoulder_offset
        @JsonProperty("d5")
        double d5; // 0.100
        @JsonProperty("d6")
        double d6; //: 0.100
        
        public DHParameters(){}
        
        @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public DHParameters(@JsonProperty("d1") double d1, 
                       @JsonProperty("a2") double a2, 
                       @JsonProperty("a3") double a3, 
                       @JsonProperty("d4") double d4, 
                       @JsonProperty("d5") double d5,
                       @JsonProperty("d6") double d6){
            this.d1 = d1;
            this.a2 = a2;
            this.a3 = a3;
            this.d4 = d4;
            this.d5 = d5;
            this.d6 = d6;
        }
    }
    
    public class Offsets {
        
        @JsonProperty("shoulder_offset")
        private double shoulder_offset;// 0.138 # measured from model
        @JsonProperty("elbow_offset")
        private double elbow_offset; // 0.007 # measured from model
        
        public Offsets(){}
        
        @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public Offsets(@JsonProperty("shoulder_offset") double shoulder_offset, 
                       @JsonProperty("elbow_offset") double elbow_offset){
            this.shoulder_offset = shoulder_offset;
            this.elbow_offset = elbow_offset;
        }
    }
    
    // inertia_parameters:
    public class InertialParameters {
            
        @JsonProperty("base_mass")
        double base_mass; // 4.0  # This mass might be incorrect
        @JsonProperty("shoulder_mass")
        double shoulder_mass;// 3.7000
        @JsonProperty("upper_arm_mass")
        double upper_arm_mass; // 8.3930
        @JsonProperty("upper_arm_inertia_offset")
        double upper_arm_inertia_offset; // 0.138 # measured from model
        @JsonProperty("forearm_mass")
        double forearm_mass; // 2.2750
        @JsonProperty("wrist_1_mass")
        double wrist_1_mass; // 1.2190
        @JsonProperty("wrist_2_mass")
        double wrist_2_mass; // 1.2190
        @JsonProperty("wrist_3_mass")
        double wrist_3_mass; // 0.1879

        @JsonProperty("shoulder_radius")
        double shoulder_radius;// 0.060   # manually measured
        @JsonProperty("upper_arm_radius")
        double upper_arm_radius; // 0.054  # manually measured
        @JsonProperty("elbow_radius")
        double elbow_radius;// 0.060      # manually measured
        @JsonProperty("forearm_radius")
        double forearm_radius;// 0.040    # manually measured
        @JsonProperty("wrist_radius")
        double wrist_radius; // 0.045      # manually measured
        
        @JsonProperty("links")
        public Map<String, Link> links = new LinkedHashMap<>();
        
        @JsonProperty("center_of_mass")
        public Map<String, XYZ> centerOfMasses = new LinkedHashMap<>();
        
        public InertialParameters(){}
        
        @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public InertialParameters(@JsonProperty("base_mass") double base_mass, 
                       @JsonProperty("shoulder_mass") double shoulder_mass,
                       @JsonProperty("upper_arm_mass") double upper_arm_mass,
                       @JsonProperty("upper_arm_inertia_offset") double upper_arm_inertia_offset,
                       @JsonProperty("forearm_mass") double forearm_mass,
                       @JsonProperty("wrist_1_mass") double wrist_1_mass,
                       @JsonProperty("wrist_2_mass") double wrist_2_mass,
                       @JsonProperty("wrist_3_mass") double wrist_3_mass,
                       @JsonProperty("shoulder_radius") double shoulder_radius,
                       @JsonProperty("upper_arm_radius") double upper_arm_radius,
                       @JsonProperty("elbow_radius") double elbow_radius,
                       @JsonProperty("forearm_radius") double forearm_radius,
                       @JsonProperty("wrist_radius") double wrist_radius){
            this.base_mass = base_mass;
            this.shoulder_mass = shoulder_mass;
            this.upper_arm_mass = upper_arm_mass;
            this.upper_arm_inertia_offset = upper_arm_inertia_offset;
            this.forearm_mass = forearm_mass;
            this.wrist_1_mass = wrist_1_mass;
            this.wrist_2_mass = wrist_2_mass;
            this.wrist_3_mass = wrist_3_mass;
            this.shoulder_radius = shoulder_radius;
            this.upper_arm_radius = upper_arm_radius;
            this.elbow_radius = elbow_radius;
            this.forearm_radius = forearm_radius;
            this.wrist_radius = wrist_radius;
        }
        
        @JsonAnySetter
        public void add(String key, Object obj) {
            if (obj instanceof Link link){
                links.put(key, link);
            } else if (obj instanceof XYZ xyz){
                centerOfMasses.put(key, xyz);
            }
        }
        /*@JsonAnySetter
        public void addXYZ(String key, XYZ xyz) {
            centerOfMasses.put(key, xyz);
        }*/
    }
    
    public static class Link {
        
        @JsonProperty("radius")
        double radius;
        @JsonProperty("length")
        double length;
        
        public Link(){}
        
        @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public Link(@JsonProperty("radius") double radius, 
                    @JsonProperty("length") double length){
            this.radius = radius;
            this.length = length;
        }
    }
    
    public static class XYZ {
        
        @JsonProperty("x")
        double x;
        @JsonProperty("y")
        double y;
        @JsonProperty("z")
        double z;
        
        public XYZ(){}
        
        @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
        public XYZ(@JsonProperty("x") double x, 
                   @JsonProperty("y") double y,
                   @JsonProperty("z") double z){
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }
}
