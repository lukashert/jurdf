package de.orat.math.xml.urdf.json;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.Chain.RPY;
import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.api.Joint;
import static de.orat.math.xml.urdf.json.RobotParameters.load;
import java.io.IOException;
import java.net.URISyntaxException;
import org.jogamp.vecmath.Vector3d;
import org.junit.jupiter.api.Test;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Test2 {
    
    public Test2() {
    }

    @Test
    public void testReadJson() throws IOException, URISyntaxException {
        RobotParameters robotParameters = RobotParameters.load("UR5e.json");
        robotParameters.save("test.json");
    }
    
    @Test
    public void testCreateUR5eURDFFromJson() throws IOException{
        RobotParameters robotParameters = RobotParameters.load("UR5e.json");
        String[] linkNames = new String[]{"base_link_inertia", "shoulder_link",
        "upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link" };
        String[] jointNames = new String[]{"shoulder_pan_joint",
            "shoulder_lift_joint","elbow_joint","wrist_1_joint", "wrist_2_joint",
        "wrist_3_joint"};
        
        Chain chain = robotParameters.createChain("robot", 
                             linkNames, jointNames, false);
        // links zu create by hand "world", "base_link"
        // joints to create by hand: "base_joint","base_link-base_link_inertia",
        de.orat.math.xml.urdf.api.Link parentLink = new de.orat.math.xml.urdf.api.Link("world");
        chain.addLink(parentLink);
        de.orat.math.xml.urdf.api.Link childLink = new de.orat.math.xml.urdf.api.Link("base_link");
        chain.addLink(childLink);
        Vector3d axis = new Vector3d(0d,0d,1d);
        Chain.RPYXYZ rpyxyz = new RPYXYZ(new RPY(0,0,0), new Vector3d(0,0,0)); 
        Joint joint = new Joint("base_joint", parentLink, childLink, axis, rpyxyz, Joint.JointType.fixed);
        chain.addJoint(joint);
        //'base_link' is REP-103 aligned (so X+ forward), while the internal
        // frames of the robot/controller have X+ pointing backwards.
        // Use the joint between 'base_link' and 'base_link_inertia' (a dummy
        // link/frame) to introduce the necessary rotation over Z (of pi rad).
        // <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
        rpyxyz = new RPYXYZ(new RPY(0d, 0d, 3.141592653589793), new Vector3d());
        joint = new Joint("base_link-base_link_inertia", childLink, 
                chain.getLink("base_link_inertia"), axis, rpyxyz, Joint.JointType.fixed);
        chain.addJoint(joint);
        chain.init();
        System.out.println("-------------- json to urdf -----------------");
        System.out.println(chain.toURDF());
    }
}
