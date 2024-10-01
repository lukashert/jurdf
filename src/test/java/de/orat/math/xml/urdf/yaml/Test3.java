package de.orat.math.xml.urdf.yaml;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import java.io.IOException;

/**
 *
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Test3 {
    
    public Test3() {
    }

    /*@Test
    public void testReadYaml() throws IOException {
        Kinematics kinematics = KinematicsWrapper.load("default_kinematics_1.yaml");
        
        KinematicsWrapper.save("test.yml");
    }*/
    
    @Test
    public void testReadYaml2() throws IOException {
        PhysicalParameters physicalParameters = PhysicalParameters.load("physical_parameters.yaml");
        
        //System.out.println(kinematics.toString());
        PhysicalParameters.save("test.yml", physicalParameters);
    }
}
