package de.orat.math.xml.urdf;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.orat.math.xml.urdf.api.Urdf;
import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.Chain.DH;
import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.json.DHParameter;
import de.orat.math.xml.urdf.json.Dynamics;
import de.orat.math.xml.urdf.json.Link;
import de.orat.math.xml.urdf.json.MechanicalJointLimits;
import de.orat.math.xml.urdf.json.PlannerConfiguration;
import de.orat.math.xml.urdf.json.SoftJointLimits;
import de.orat.math.xml.urdf.util.DHUtils2;
import de.orat.math.xml.urdf.util.RotationUtils;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import javax.xml.parsers.ParserConfigurationException;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Vector3d;
import org.junit.jupiter.api.Test;
import org.xml.sax.SAXException;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Test1 {
    
    private static List<DH> dhRRPRRobot = new ArrayList<>();
    private static List<DH> dhUR5eWandelbots = new ArrayList<>();
    private static List<DH> dhUR5Nominal = new ArrayList<>();
    private static List<DH> dhUR5eNominal = new ArrayList<>();
    
    private static List<DH> dhTest = new ArrayList<>();
    
    private static List<DH> dhTest2 = new ArrayList<>();
    
    public Test1() {
        
        dhRRPRRobot.clear();
        dhUR5eWandelbots.clear();
        dhUR5Nominal.clear();
        dhUR5eNominal.clear();
        dhTest.clear();
        dhTest2.clear();
        
        // double d, double theta, double r, double alpha
        dhRRPRRobot.add(new DH(0, 0, 0, 0));
        dhRRPRRobot.add(new DH(0.2, 0, 0, -Math.PI/4d));
        dhRRPRRobot.add(new DH(0, -Math.PI/4d, 0.3, 0));
        dhRRPRRobot.add(new DH(0, -Math.PI/4d, 0.2, Math.PI));
        dhRRPRRobot.add(new DH(0, 0, 0.1, 0));
        
        // UR5e Wandelbots
        dhUR5eWandelbots.add(new DH(0.089159, 0, 0, Math.PI/2));
        dhUR5eWandelbots.add(new DH(0, 0, -0.4250, 0));
        dhUR5eWandelbots.add(new DH(0, 0, -0.39225, 0));
        dhUR5eWandelbots.add(new DH(0.10915, 0, 0, Math.PI/2));
        dhUR5eWandelbots.add(new DH(0.09465, 0, 0, -Math.PI/2));
        dhUR5eWandelbots.add(new DH(0.0823, 0, 0, 0));
        
        // UR5 nominal, sollte die via urdf2casadi mitgelieferte urdf übereinstimmen
        dhUR5Nominal.add(new DH(0.089159, 0, 0, Math.PI/2));
        dhUR5Nominal.add(new DH(0,0,-0.425,0));
        dhUR5Nominal.add(new DH(0,0,-0.39225,0));
        dhUR5Nominal.add(new DH(0.10915,0,0,Math.PI/2));
        dhUR5Nominal.add(new DH(0.09465,0,0,-Math.PI/2));
        dhUR5Nominal.add(new DH(0.0823,0,0,0));
        
        dhUR5eNominal.add(new DH(0.1625, 0, 0, Math.PI/2));
        dhUR5eNominal.add(new DH(0, 0, -0.425, 0));
        dhUR5eNominal.add(new DH(0, 0, -0.3922, 0));
        dhUR5eNominal.add(new DH(0.1333, 0, 0, Math.PI/2));
        dhUR5eNominal.add(new DH(0.0997, 0, 0, -Math.PI/2));
        dhUR5eNominal.add(new DH(0.0996, 0, 0, 0));
       
        dhTest.add(new DH(4,1,0,Math.PI/2d )); // link 1
        dhTest.add(new DH(0,2,5,0 )); // link 2
        dhTest.add(new DH(0,3,6,0 )); // link 3
        
        dhTest2.add(new DH(4,1,0,Math.PI/2d )); // link 1
        dhTest2.add(new DH(0,2,5,0)); // link 2
        dhTest2.add(new DH(0,3,6,0 )); // link 3
        
        // urdf-->dh hat folgendes ergeben
        // - urdf wurde korrekt geladen als rpyxyz aller joints sind korrekt
        // - 3 Zeilen ist korrekt, also eigentlich eine mehr als zu erwarten
        // - pi/2 ist eine Zeile verrutscht
        // - theta-Werte fehlen alle, das ist korrekt
        /*link-name, d, theta, r, alpha
            link1, 4.0, 0.0, 0.0, 0.0
            link2, 0.0, 0.0, 5.0, 1.570706326
            link3, 0.0, 0.0, 6.0, 0.0*/
     

        // das wäre richtig
        // urdf
        // xyz rpy
        // joint Base/link 1: 0 0 4, 0,0,0
        // joint 1:  5 0 0,  1.570706326, 0 0
        // joint 2:  6 0 0,  0 0 0
        // joint 3:  0 0 0,  0 0 0
        
     
    }

    // scheint vollkommen falsch zu sein
    // 2. und dritte axis ist komplett falsch
    // rpy sind alle 0, da sollte beim ersten joint ein pi/2 drinstehen
    @Test
    public void testToChain(){
        DH[] dhParams = dhUR5eNominal.toArray(new DH[]{}); //new DH[]{new DH(0d,0d,0.1625, Math.PI/2d)};
        Chain chain = Urdf.toChainOld("Test", dhParams, null, false);
        System.out.println("-------- test to chain ------------");
        System.out.println(chain.toURDF());
        System.out.println("-----------------------------------");
        
        /* ur5enominal mit toChainOld()
        - alle Winkel sind fälschlicherweise 0
        <joint name="alink_0_to_link_0" type="revolute"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
	<joint name="link_0_to_alink_1" type="fixed"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/><axis xyz="0.0 0.0 0.0"/>
        <joint name="alink_1_to_link_1" type="revolute"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1625"/><axis xyz="0.0 -1.0 6.123233995736766E-17"/>
        <joint name="link_1_to_alink_2" type="fixed"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/><axis xyz="0.0 0.0 0.0"/>
        <joint name="alink_2_to_link_2" type="revolute"><origin rpy="0.0 0.0 0.0" xyz="-0.425 -0.0 0.0"/><axis xyz="0.0 -1.0 6.123233995736766E-17"/>
        <joint name="link_2_to_alink_3" type="fixed"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/><axis xyz="0.0 0.0 0.0"/>
        <joint name="alink_3_to_link_3" type="revolute"><origin rpy="0.0 0.0 0.0" xyz="-0.3922 -0.0 0.0"/><axis xyz="0.0 -1.0 6.123233995736766E-17"/>
        <joint name="link_3_to_alink_4" type="fixed"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/><axis xyz="0.0 0.0 0.0"/>
        <joint name="alink_4_to_link_4" type="revolute"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1333"/><axis xyz="0.0 -1.2246467991473532E-16 -1.0"/>
        <joint name="link_4_to_alink_5" type="fixed"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/><axis xyz="0.0 0.0 0.0"/>
        <joint name="alink_5_to_link_5" type="revolute"><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0997"/><axis xyz="0.0 -1.0 6.123233995736766E-17"/>
        */
    }
    
    @Test
    public void dhUR5eNominalToChain(){
        DH[] dhParams = dhUR5eNominal.toArray(new DH[]{}); //new DH[]{new DH(0d,0d,0.1625, Math.PI/2d)};
        Chain chain = Urdf.toChainOld("Test", dhParams, null, false);
        System.out.println("------ dhUR5eNominal to chain ------------");
        System.out.println(chain.toURDF());
        System.out.println("-----------------------------------");
    }
    
    @Test
    public void testURDF2DH() throws SAXException, IOException, ParserConfigurationException {
        Urdf jurdf = new Urdf(getClass().getResourceAsStream("ur5.urdf"));
        Chain robot = jurdf.createChain();
        System.out.println(robot.toURDF());
        Map<String, DH>  dhList = robot.toDH();
        System.out.println("------------- test ur5.urdf to DH ---------------------");
        System.out.println("link-name, d, theta, r, alpha");
        for (String linkName: dhList.keySet()){
            DH dh = dhList.get(linkName);
            System.out.println(linkName+", "+String.valueOf(dh.d())+", "+String.valueOf(dh.theta()+", "
            +String.valueOf(dh.r())+", "+String.valueOf(dh.alpha())));
        }
        System.out.println("----------------------------------------------------");
    }
    
    @Test 
    public void testDHRPYXYZ(){
        for (DH dh: dhUR5eNominal){
            Matrix4d dhFrame = RotationUtils.to(dh);
            RPYXYZ rpyxyz = RotationUtils.toRPYXYZ(dhFrame, false);
            System.out.println(Chain.toXML(rpyxyz));
        }
        System.out.println("-------");
        for (DH dh: dhUR5eNominal){
            Matrix4d dhFrame = RotationUtils.to(dh);
            RPYXYZ rpyxyz = RotationUtils.toRPYXYZ(dhFrame, true);
            System.out.println(Chain.toXML(rpyxyz));
        }
    }
    
    /*@Test
    public void testURDF2DH() throws SAXException, IOException, ParserConfigurationException {
        Urdf jurdf = new Urdf(getClass().getResourceAsStream("ur5e.urdf"));
        Chain robot = jurdf.createChain();
        System.out.println(robot.toURDF());
        HashMap<String, DH>  dhList = robot.toDH();
        System.out.println("------------- test urdf to DH ---------------------");
        System.out.println("link-name, d, theta, r, alpha");
        for (String linkName: dhList.keySet()){
            DH dh = dhList.get(linkName);
            System.out.println(linkName+", "+String.valueOf(dh.d())+", "+String.valueOf(dh.theta()+", "
            +String.valueOf(dh.r())+", "+String.valueOf(dh.alpha())));
        }
        System.out.println("----------------------------------------------------");
    }*/
    
    // keine gute Übereinstimmung, da das Vergleichs urdf-file Gelenkachsen in 
    // y-Richtung verwendet.
    @Test
    public void testDHUR52RPYXYZ(){
        System.out.println("-------------- test_DH_UR5_2_RPYXYZ ------------------");
        List<Matrix4d> mats = DHUtils2.to(dhUR5Nominal, false);
        for (Matrix4d m: mats){
            RPYXYZ rpyxyz = RotationUtils.toRPYXYZ(m, false);
            System.out.println(rpyxyz.toString());
        }
        /*
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.089159)]
        RPYXYZ[rpy=RPY[roll=1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.0)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(-0.425, 0.0, 0.0)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(-0.39225, 0.0, 0.10915)]
        RPYXYZ[rpy=RPY[roll=1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, -0.09465, 5.795640976964849E-18)]
        RPYXYZ[rpy=RPY[roll=-1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0823, 5.039421578491359E-18)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.0)]
  
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.089159"/>
    <axis xyz="0 0 1"/>

    <origin rpy="0.0 1.57079632679 0.0" xyz="0.0 0.13585 0.0"/>
    <axis xyz="0 1 0"/>

    <origin rpy="0.0 0.0 0.0" xyz="0.0 -0.1197 0.425"/>
    <axis xyz="0 1 0"/>

    <origin rpy="0.0 1.57079632679 0.0" xyz="0.0 0.0 0.39225"/>
    <axis xyz="0 1 0"/>
  
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.093 0.0"/>
    <axis xyz="0 0 1"/>
   
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.09465"/>
    <axis xyz="0 1 0"/>
  <origin rpy="-1.57079632679 0 0" xyz="0 0.0823 0"/>
     
        
        */
        System.out.println("------------------------------------------------");
    }
    
    // da stehen die test-Daten für xacro basiertes urdf-file Erzeugung
    // /opt/ros/iron/share/ur_description/urdf
    //TODO
    // zum Laufen bringen
    // Achtung: Korrektur wirkt sich nicht aus wegen paralleler Achsen bei ur5e,
    // erst bei korrigierten dh-parametern mit nicht parallelen Achsen kommt es zu
    // Unterschieden. 
    // Ursache? vielleicht da die Multiplikationen nicht in der richtigen Reihenfolge
    // stattfinden bzw. anders zusammengezogen werden? siehe paper mit entsprechender
    // Algebra?
    @Test
    public void testDH2RPYXYZ(){
        System.out.println("-------------- testDHUR5e_2_RPYXYZ ------------------");
        List<Matrix4d> mats = DHUtils2.to(dhUR5eNominal, false);
        for (Matrix4d m: mats){
            RPYXYZ rpyxyz = RotationUtils.toRPYXYZ(m, false);
            System.out.println(rpyxyz.toString());
        }
        System.out.println("------------------------------------------------");
        
        /*
        TODO kontrollieren ob das übereinstimmt mit dem erzeugen urdf file, eventuell
        überprüfen, ob die input-dh-parameter auch übereinstimmen mit denen aus denen
        das urdf-file erzeugt wird... 
        
        mit correction
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.1625)]
        RPYXYZ[rpy=RPY[roll=1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.0)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(-0.425, 0.0, 0.0)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(-0.3922, 0.0, 0.1333)]
        RPYXYZ[rpy=RPY[roll=1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, -0.0997, 6.1048642937495555E-18)]
        RPYXYZ[rpy=RPY[roll=-1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0996, 6.098741059753818E-18)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.0)]
        
        ohne correction
        // 
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.1625)]
        RPYXYZ[rpy=RPY[roll=1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.0)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(-0.425, 0.0, 0.0)]
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(-0.3922, 0.0, 0.1333)]
        RPYXYZ[rpy=RPY[roll=1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, -0.0997, 6.1048642937495555E-18)]
        // im folgenden steht noch 2x pi für pitch und yaw im vergleichs urdf
        RPYXYZ[rpy=RPY[roll=-1.5707963267948966, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0996, 6.098741059753818E-18)]
        // den letzten gibts im vergleichs urdf nicht
        RPYXYZ[rpy=RPY[roll=0.0, pitch=-0.0, yaw=0.0], xyz=(0.0, 0.0, 0.0)]
        */
        
        /*
        so stehts im urdf, Variante ohne passive joints d.h. simplified!!!
        
   <joint name="base_joint" type="fixed"><origin rpy="0 0 0" xyz="0 0 0"/>
   <joint name="base_link-base_link_inertia" type="fixed"><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
        
   stimmt grundsätzlich überein bis auf wrist_3
   <joint name="shoulder_pan_joint" type="revolute"><origin rpy="0 0 0" xyz="0 0 0.1625"/>
   <joint name="shoulder_lift_joint" type="revolute"><origin rpy="1.570796327 0 0" xyz="0 0 0"/>
   <joint name="elbow_joint" type="revolute"><origin rpy="0 0 0" xyz="-0.425 0 0"/>
   <joint name="wrist_1_joint" type="revolute"><origin rpy="0 0 0" xyz="-0.3922 0 0.1333"/>
   <joint name="wrist_2_joint" type="revolute"><origin rpy="1.570796327 0 0" xyz="0 -0.0997 -2.044881182297852e-11"/>
   <joint name="wrist_3_joint" type="revolute"><origin rpy="1.570796326589793 3.141592653589793 3.141592653589793" xyz="0 0.0996 -2.042830148012698e-11"/>
   
   <joint name="wrist_3_link-ft_frame" type="fixed"><origin rpy="3.141592653589793 0 0" xyz="0 0 0"/>
        
   <joint name="base_link-base_fixed_joint" type="fixed"><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
   <joint name="wrist_3-flange" type="fixed"><origin rpy="0 -1.5707963267948966 -1.5707963267948966" xyz="0 0 0"/>
   <joint name="flange-tool0" type="fixed"><origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0 0 0"/>
    
        */
    }
    
    
    @Test
    public void testDH2RPYXYZ2(){
        System.out.println("-------------- testDH2RPYXYZ 2 ------------------");
        // beinhaltet correction of segment 1,2 wie bei UR5e, Voraussetzung dazu
        // ist dass d1=d2=0, d0!=0
        List<Matrix4d> mats = DHUtils2.to(dhTest2, false);
        
        for (Matrix4d m: mats){
            RPYXYZ rpyxyz = RotationUtils.toRPYXYZ(m, false);
            System.out.println(rpyxyz.toString());
        }
        System.out.println("------------------------------------------------");
    }    
        
    @Test
    public void testextractVector(){
        Matrix4d m = new Matrix4d();
        m.setIdentity();
        m.m03 = 3;
        Vector3d trans = new Vector3d();
        m.get(trans);
        System.out.println(Chain.toString(m));
        System.out.println("x="+trans.x);
        System.out.println("y="+trans.y);
        System.out.println("z="+trans.z);
    }
    
    // scheint so zu stimmen für alle theta == 0!!!
    // für theta2 != 0 scheint es auch zu stimmen!!!
    @Test
    public void testUR5eFK(){
        List<Matrix4d> simplified_chain = DHUtils2.to(dhUR5eNominal, false);
        double[] joint_values = new double[]{0d,1.58824962d,0d,0d,0d,0d};
        int link_nr = 6;
        Matrix4d m = DHUtils2.calcForwardKinematics(simplified_chain, joint_values, link_nr);
        System.out.println("----------  testUR5eFK---------");
        System.out.println(m.toString());
    }
    
    public static String toString(Matrix4d m){
        StringBuilder sb = new StringBuilder();
        sb.append(String.valueOf(m.m00));
        sb.append("|");
        sb.append(String.valueOf(m.m01));
        sb.append("|");
        sb.append(String.valueOf(m.m02));
        sb.append("|");
        sb.append(String.valueOf(m.m03));
        sb.append("\n");
        sb.append(String.valueOf(m.m10));
        sb.append("|");
        sb.append(String.valueOf(m.m11));
        sb.append("|");
        sb.append(String.valueOf(m.m12));
        sb.append("|");
        sb.append(String.valueOf(m.m13));
        sb.append("\n");
        sb.append(String.valueOf(m.m20));
        sb.append("|");
        sb.append(String.valueOf(m.m21));
        sb.append("|");
        sb.append(String.valueOf(m.m22));
        sb.append("|");
        sb.append(String.valueOf(m.m23));
        sb.append("\n");
        sb.append(String.valueOf(m.m30));
        sb.append("|");
        sb.append(String.valueOf(m.m31));
        sb.append("|");
        sb.append(String.valueOf(m.m32));
        sb.append("|");
        sb.append(String.valueOf(m.m33));
        sb.append("\n");
        return sb.toString();
    }
}
