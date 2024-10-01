package de.orat.math.xml.urdf.yaml;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import java.io.IOException;
import java.io.InputStream;

/**
 *
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class KinematicsWrapper {
    
    @JsonProperty("kinematics")
    private static Kinematics kinematics;
    
    // INDENT_OUTPUT sorgt für Zeilenumbrüche
    private static ObjectMapper objectMapper = (new ObjectMapper(new YAMLFactory()))/*.enable(SerializationFeature.INDENT_OUTPUT)*/;
   
    public KinematicsWrapper(){}
    
    public static Kinematics load(String yamlFileName) throws IOException{
        InputStream inputStream = Kinematics.class.getResourceAsStream(yamlFileName);
        kinematics = objectMapper.readValue(inputStream, Kinematics.class);
        return kinematics;
    }
    
    public static void save(String yamlFileName) throws JsonProcessingException{
        String parsedJson = objectMapper.writeValueAsString(kinematics);
        //TODO in Datei schreiben
        System.out.println(parsedJson);
    }
    
}
