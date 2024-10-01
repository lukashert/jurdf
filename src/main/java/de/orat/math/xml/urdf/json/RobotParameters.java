package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.Chain.DH;
import de.orat.math.xml.urdf.api.Joint;
import de.orat.math.xml.urdf.api.Joint.JointType;
import de.orat.math.xml.urdf.util.DHUtils2;
import de.orat.math.xml.urdf.util.RotationUtils;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import lombok.Getter;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 * 
 * TODO
 * Wie kann ich dafür sorgen dass alle Properties beim Rausschreiben in Strings
 * umgewandelt werden und damit in "" gekapselt werden?
 * 
 */
@Getter  
public class RobotParameters {
    
    // JsonProperty sorgt dafür dass beim Serialisieren die angegebenen Namen
    // verwendet werden und nicht die Variablen-Name (in Kleinbuchstaben)
    @JsonProperty("DHParameter")
    private final List<DHParameter> dhParameter;
    @JsonProperty("MechanicalJointLimits")
    private final List<MechanicalJointLimits> mechanicalJointLimits;
    @JsonProperty("SoftJointLimits")
    private final List<SoftJointLimits> softJointLimits;
    @JsonProperty("Links")
    private final List<Link> links;
    @JsonProperty("Dynamics")
    private final List<Dynamics> dynamics;
    @JsonProperty("PlannerConfiguration")
    private final PlannerConfiguration plannerConfiguration;
    
    // INDENT_OUTPUT sorgt für Zeilenumbrüche
    private static ObjectMapper objectMapper = (new ObjectMapper()).enable(SerializationFeature.INDENT_OUTPUT);
            
    @JsonCreator(mode = JsonCreator.Mode.PROPERTIES)
    public RobotParameters(@JsonProperty("DHParameter") List<DHParameter> dhParameter, 
            @JsonProperty("MechanicalJointLimits") List<MechanicalJointLimits> mechanicalJointLimits,
            @JsonProperty("SoftJointLimits") List<SoftJointLimits> softJointLimits, 
            @JsonProperty("Links") List<Link> links, 
            @JsonProperty("Dynamics") List<Dynamics> dynamics, 
            @JsonProperty("PlannerConfiguration") PlannerConfiguration plannerConfiguration ){
        this.dhParameter = dhParameter;
        this.mechanicalJointLimits = mechanicalJointLimits;
        this.softJointLimits = softJointLimits;
        this.links = links;
        this.dynamics = dynamics;
        this.plannerConfiguration = plannerConfiguration;
    }
    
    public static RobotParameters load(String jsonFileName) throws IOException {
        try {
            byte[] jsonData = Files.readAllBytes(Paths.get(RobotParameters.class.getResource(jsonFileName).toURI()));
            return objectMapper.readValue(jsonData, RobotParameters.class);
        } catch (URISyntaxException ex) {
            Logger.getLogger(RobotParameters.class.getName()).log(Level.SEVERE, null, ex);
        }
        return null;
    }
    public void save(String jsonFileName) throws JsonProcessingException{
        var parsedJson = objectMapper.writeValueAsString(this);
        System.out.println(parsedJson);
    }
    
    public List<DH> getDHList(){
        List<DH> result = new ArrayList<>(); 
        for (DHParameter dh: dhParameter){
            result.add(dh.toDH());
        }
        return result;
    }
    @Deprecated
    public DH[] getDH(){
        DH[] result = new DH[dhParameter.size()];
        int i=0;
        for (DHParameter dh: dhParameter){
            result[i++] = dh.toDH();
        }
        return result;
    }
    /**
     * Create an array of urdf chain links.
     * 
     * @param linkNames
     * @return 
     */
    public de.orat.math.xml.urdf.api.Link[] getURDFLinks(String[] linkNames){
        de.orat.math.xml.urdf.api.Link[] result = new de.orat.math.xml.urdf.api.Link[linkNames.length];
        result[0] = new de.orat.math.xml.urdf.api.Link(linkNames[0]);
        for (int i=1;i<linkNames.length;i++){
            result[i] = links.get(i-1).toURDFLink(linkNames[i]);
        }
        return result;
    }
        
    /**
     * Create an urdf-chain based on RobotParameters.
     * 
     * @param robotParametersJsonFileName
     * @param withFakeLinks true if fake-links corresponding to the joints should be created
     * @return chain of links and joints
     * @throws IOException 
     */
    public Chain createChain(String robotName, 
                             String[] linkNames, String[] revoluteJointNames, 
                             boolean withFakeLinks) throws IOException {
        Chain robot = new Chain(robotName);
        List<DH> segments = getDHList();
        List<Matrix4d> mList = DHUtils2.to(segments, withFakeLinks);
        de.orat.math.xml.urdf.api.Link[] mylinks = getURDFLinks(linkNames);
        robot.addLink(mylinks[0]);
        for (int i=1;i<mylinks.length;i++){
            if (mylinks[i] == null) throw new RuntimeException("Link "+String.valueOf(i)+" is null!");
            robot.addLink(mylinks[i]);
            String jointName = revoluteJointNames[i-1]; 
            Vector3d zAxis = new Vector3d(0d,0d,1d);
            Chain.RPYXYZ rpyxyz = RotationUtils.toRPYXYZ(mList.get(i-1), false);
            Joint joint = new Joint(jointName, mylinks[i-1], mylinks[i], 
                    zAxis, rpyxyz, JointType.revolute);
            robot.addJoint(joint);
            if (withFakeLinks){
                //TODO
                // joint names aus parent- und child-link name mit Bindestrich
                // zusammensetzen
            }
        }
        return robot;
    }
}
