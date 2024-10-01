package de.orat.math.xml.urdf.api;

import de.orat.math.xml.urdf.util.DHUtils;
import de.orat.math.xml.urdf.util.LineUtils;
import de.orat.math.xml.urdf.util.RotationUtils;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Vector3d;

/**
 * A manipulator is a serie of links (rigid bodys) connected by joints
 * (essentially prismatic or revolute) forming a kinematic chain between the base
 * and the end-effector.
 * 
 * TODO
 * besser in Tree umbenennen? denn mehrere joints pro link sind ja möglich
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Chain {
    
    // angles in [rad] and other parameters in [m]
    public record DH(double d, double theta, double r, double alpha){};
    public record mDH(double d, double theta, double r, double alpha){};
    
    // Represents the rotation around fixed axis: first roll around x, then pitch 
    // around y and finally yaw around z. All angles are specified in radians.
    // Fixed axis rotation also includes Euler extrinsic rotation around fixed axis, 
    // like RPY around fixed X-Y-Z used below
    public record RPY(double roll, double pitch, double yaw){};
    // unit [m], [rad]
    // from the parent to the child, translation is applied first, followed by 
    // fixed-axis rotations.
    public record RPYXYZ(RPY rpy, Vector3d xyz){};
    
    private String name;
    
    private Map<String, Link> links = new HashMap<>();
    private Map<String, Joint> joints = new HashMap<>();
    
    private Link rootLink;
    
    public record CollisionSphere(Point3d position, double radius){};
    
    public Chain(String name){
        this.name = name;
    }
    
    public Link getRootLink(){
        return rootLink;
    }
    public void addLink(Link link){
        links.put(link.getName(), link);
    }
    public void addJoint(Joint joint){
        joints.put(joint.getName(), joint);
        joint.getChild().setParent(joint);
        joint.getParent().addChild(joint);
    }
    
    public Link getLink(String linkName){
        return links.get(linkName);
    }
    
    public void init(){
        findRootLink();
        calcTFs();
    }
    void findRootLink(){
        Collection<Link> linkObjs = links.values();
        int foundRootLinks = 0;
        StringBuilder sb = new StringBuilder();
        sb.append("Found root links in arbitrary order:\n");
        for (Link link: linkObjs){
            if (link.getParent() == null){
                sb.append("rootlink \"");
                sb.append(link.getName());
                sb.append("\" found!\n");
                rootLink = link;
                foundRootLinks++;
            }
        }
        switch (foundRootLinks){
            case 0:
                throw new RuntimeException("No root link found!");
            case 1:
                return;
            default:
                System.out.println(sb.toString());
                throw new RuntimeException(String.valueOf(foundRootLinks)+" root links found!");
        }
    }
    
    /**
     * Get collision spheres.
     * 
     * @param jointName
     * @return coordinates in the corresponding base joint coordinate system
     */
    public Collection<CollisionSphere> collisionSpheres(String jointName){
        // Möglichkeit A
        // 1. Die referenzierten stl-files nehmen und eine umhüllenden Kugel bestimmen
        // 2. link-length bestimmen, Radius obiger Kugel abziehen und entsprechend auch
        // von anderen Ende und dann restlichen Link mit Kugeln auffüllen, Radius dieser
        // Kugeln bestimmt sich irgendwie aus dem stl oder wird erst mal von hand
        // vorgegeben.
        // Möglichkeit B
        // Collision elements verwenden, von Cylindern ausgehen und diese in Kugeln
        // umwandeln
        return null;
    }
    
    /**
     * Calculate technical frames (absolute and relative) for the links in the 
     * world coordinate system and save it in the link objects.
     */
    void calcTFs(){
        System.out.println("calcTFs with rootLink = "+rootLink.getName());
        Collection<Joint> rootLinkJoints = rootLink.getChildren();
        for (Joint joint: rootLinkJoints){
            System.out.println("  calcTFs joint="+joint.getName()+" childlink="+joint.getChild().getName());
            calcTFRecursive(joint.getChild());
        }
    }
    
    /**
     * Calculate and save into the link objects recursively, relative and 
     * absolute frames.
     * 
     * This is absTF and relTF in the link elements.<p>
     * 
     * @see calculate_tfs_in_world_frame
     * @precondition beim root-link muss absTF gesetzt sein
     * @param link link with parent link (the root-link as argument is not allowed here)
     * @throws IllegalArgumentException if the given link is the root link (the 
     * given link has no parent link
     */
    private void calcTFRecursive(Link link){
        if (link == null) throw new IllegalArgumentException("link==null is not allowed!");
        if (link.getParentLink() == null) throw new IllegalArgumentException("link has no parent link!");
        System.out.println("    calcTFRecursive link="+link.getName());
        
        Joint joint = link.getParent();
        Matrix4d relTF = RotationUtils.to(joint.getRPYXYZ());
        link.setRelTF(relTF);

        // parent_tf_world = self.urdf_links[n.parent.parent.id]['abs_tf']
        Link parentLink = link.getParentLink();
        Matrix4d absTF = parentLink.getAbsTF();
        if (absTF == null){
            // Current Element: type=joint, name=flange-tool0
            // Fehler: link = shoulder_link parentlink=base_link_inertia absTF==null
            System.out.println("Fehler!!!: link = "+link.getName()+" parentlink="+parentLink.getName()+" absTF==null");
            //FIXME
            // dafür sorgen dass die root links absTF mit eye belegen
            absTF = new Matrix4d();
            absTF.setIdentity();
        }
        
        absTF = new Matrix4d(absTF); // wichtig, damit das nicht mit mul() überschrieben wird
        absTF.mul(relTF);
        link.setAbsTF(absTF);
        
        Collection<Joint> childJoints = link.getChildren();
        for (Joint childJoint: childJoints){
            Link childLink = childJoint.getChild();
            if (childLink != null) calcTFRecursive(childLink);
        }      
    }
    
    
    /**
     * Get corresponding DH parameters if possible.
     * 
     * Conversion into DH is not possible in all cases. Especially DH parameters 
     * do support joint rotatation around the z-axis only.<p>
     * 
     * Passive/fix joints are ignored. Joints are ordered by hierarchy dependency. 
     * By convention N joints give N-1 DH parameters, but this method returns N 
     * parameters. The reason is because the first parameter is used to define 
     * the coordinate system of the first axis relative to the robot origin.<p>
     * 
     * TODO
     * Umgang mit trees, also mehrere joint bei einem link, vermutlich brauche
     * ich dann als zusätzliches Argument, die letzten Blätter/end-effectors damit
     * ich dann für jede zugehörige chain einen Satz dh-parameter bestimmen kann
     * 
     * @see def calculate_dh_params(self):
     * @return key==link-name
     */
    public Map<String, DH> toDH(){
        Map<String, DH> result = new LinkedHashMap<>();
        Collection<Joint> rootLinkJoints = rootLink.getChildren();
        for (Joint joint: rootLinkJoints){
            calcDHRecursive(joint.getChild(), result);
        }
        return result;
    }
    
    /**
     * Calculate and save dh-frames and determine the dh-parameters.
     * 
     * https://github.com/mcevoyandy/urdf_to_dh/blob/main/urdf_to_dh/generate_dh.py<p>
     * 
     * @precondition absTF, absDHTF
     * @param link not the root link
     * @param dhParameters result dh parameters
     */
    private void calcDHRecursive(Link link, Map<String, DH> dhParameters){
        //for urdf_node in LevelOrderIter(self.root_link):
        //if urdf_node.type == 'link' and self.urdf_links[urdf_node.id]['dh_found'] == False:
        System.out.println("\n\nprocess dh params for "+link.getName());

        // TF from current link frame to world frame
        // link_to_world = self.urdf_links[urdf_node.id]['abs_tf']
        Matrix4d link_to_world = link.getAbsTF();

        // DH frame from parent link frame to world frame
        // parent_to_world_dh = self.urdf_links[urdf_node.parent.parent.id]['abs_dh_tf']
        Joint parentJoint = link.getParent();
        Matrix4d parent_to_world_dh = parentJoint.getParent().getAbsDHTF();
        

        // TF from link frame to parent dh frame
        // link_to_parent_dh = np.matmul(kh.inv_tf(parent_to_world_dh), link_to_world)
        Matrix4d link_to_parent_dh = new Matrix4d(parent_to_world_dh);
        System.out.println("parent link="+parentJoint.getParent().getName());
        System.out.println("parent_to_world_dh:");
        System.out.println(toString(parent_to_world_dh));
        link_to_parent_dh.invert();
        link_to_parent_dh.mul(link_to_world);

        // Find DH parameters
        //axis = np.matmul(link_to_parent_dh[0:3, 0:3], self.urdf_joints[urdf_node.parent.id]['axis'])
        Vector3d axis = parentJoint.getAxis();
        //FIXME
        // warum kann axis== null sein? base_link hat keine Achse
        // ist transform hier überhaupt richtig?
        link_to_parent_dh.transform(axis);
        
        //dh_params = self.get_joint_dh_params(link_to_parent_dh, axis)
        DH dhParams = DHUtils.toDH(link_to_parent_dh, axis);
        
        //dh_frame = kh.get_dh_frame(dh_params)
        Matrix4d dh_frame = RotationUtils.to(dhParams);
                
        //abs_dh_frame = np.matmul(parent_to_world_dh, dh_frame)
        Matrix4d abs_dh_frame = new Matrix4d(parent_to_world_dh);
        abs_dh_frame.mul(dh_frame);
        
        //self.urdf_links[urdf_node.id]['dh_tf'] = dh_frame
        //link.setDHTF(dh_frame);

        //self.urdf_links[urdf_node.id]['abs_dh_tf'] = abs_dh_frame
        link.setAbsDHTF(abs_dh_frame);
        
        //robot_dh_params.append([urdf_node.parent.id, urdf_node.parent.parent.id, urdf_node.id] + list(dh_params.round(5)))
        dhParameters.put(link.getName(), dhParams);

        //Link childLink = link.getChildLink();
        for (Joint joint: link.getChildren()){
            Link childLink = joint.getChild();
            if (childLink != null) calcDHRecursive(childLink, dhParameters);
        }
    }
    
    public String toURDF(){
        StringBuilder sb = new StringBuilder();
        sb.append("<?xml version=\"1.0\" ?>\n");
        sb.append("<robot name=\"");
        sb.append(name);
        sb.append("\">\n");
        if (rootLink == null) throw new RuntimeException("Root link is null!");
        add2URDFRecursive(sb, rootLink);
        sb.append("</robot>");
        return sb.toString();
    }
    private void add2URDFRecursive(StringBuilder sb, Link link){
        sb.append("  ");
        sb.append(link.toXML());
        List<Joint> myjoints = link.getChildren();
        for (Joint joint: myjoints){
            sb.append("    ");
            sb.append(joint.toXML());
            Link childLink = joint.getChild();
            if (childLink != null){
                add2URDFRecursive(sb, childLink);
            }
        }
    }
    
    public static String toXML(RPYXYZ rpyxyz){
        StringBuilder sb = new StringBuilder();
        sb.append("<origin rpy=\"");
        sb.append(String.valueOf(rpyxyz.rpy().roll()));
        sb.append(" ");
        sb.append(String.valueOf(rpyxyz.rpy().pitch()));
        sb.append(" ");
        sb.append(String.valueOf(rpyxyz.rpy().yaw()));
        sb.append("\" xyz=\"");
        sb.append(String.valueOf(rpyxyz.xyz().x));
        sb.append(" ");
        sb.append(String.valueOf(rpyxyz.xyz().y));
        sb.append(" ");
        sb.append(String.valueOf(rpyxyz.xyz().z));
        sb.append("\"/>\n");
        return sb.toString();
    }
    
    public static String toString(Matrix4d m){
        StringBuilder sb = new StringBuilder();
        sb.append(String.valueOf(m.m00));
        sb.append(",");
        sb.append(String.valueOf(m.m01));
        sb.append(",");
        sb.append(String.valueOf(m.m02));
        sb.append(",");
        sb.append(String.valueOf(m.m03));
        sb.append("\n");
        sb.append(String.valueOf(m.m10));
        sb.append(",");
        sb.append(String.valueOf(m.m11));
        sb.append(",");
        sb.append(String.valueOf(m.m12));
        sb.append(",");
        sb.append(String.valueOf(m.m13));
        sb.append("\n");
        sb.append(String.valueOf(m.m20));
        sb.append(",");
        sb.append(String.valueOf(m.m21));
        sb.append(",");
        sb.append(String.valueOf(m.m22));
        sb.append(",");
        sb.append(String.valueOf(m.m23));
        sb.append("\n");
        sb.append(String.valueOf(m.m30));
        sb.append(",");
        sb.append(String.valueOf(m.m31));
        sb.append(",");
        sb.append(String.valueOf(m.m32));
        sb.append(",");
        sb.append(String.valueOf(m.m33));
        return sb.toString();
    }
}
