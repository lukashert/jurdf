package de.orat.math.xml.urdf.api;

import de.orat.math.xml.urdf.api.Chain.RPY;
import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.util.RotationUtils;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Vector3d;

/**
 * The joint element describes a homogeneous transformation that will take 
 * coordinates in the child link’s coordinate frame and transform them into the 
 * parent link’s coordinate frame.<p>
 *
 * The joint elements also give the axisInParentFrame of the joint with respect to the origin 
 * of the child link’s coordinate frame.
 *
 * https://mcevoyandy.github.io/urdf_to_dh/derivations.html
 *
 * Unlike DH-Parameters URDF allows free choice of frame orientation.
 *
 * https://github.com/AdoHaha/DH2URDF/tree/master
 * 
 * https://sir.upc.edu/projects/kinematics_dynamics_control_theory/foundations/robotmodelling/robotmodelling.html
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Joint {
    
    // urdf2casadi supports only prismatic, fixed and revolute
    public enum JointType {
        revolute, continuous, prismatic, fixed, floating, planar;
    }
    
    private final JointType jointType;
    
    private final String name;
    private final Link parent;
    private final Link child; 
    
    // Axis of the joint with respect to the the childs link frame
    // ROS-Doku: The joint axis is specified in the joint frame. This is the axis
    // of rotation for revolute joints, the axis of translation for prismatic joint,
    // and floating joints do not use the axis field.
    // This vector should be normalized.
    private final Vector3d axis;
    
    // ROS-Doku: The transformation from the parent link to the child link. The
    // joint is located at the origin of the child link.
    // == Transformation that locates the child link with respect to the parent 
    // link
    private RPYXYZ origin; 
    
    // default z-Achse, da für joint="base_link-base_link_inertia" in der URDF 
    // das element axisInParentFrame nicht gesetzt sein muss
    
    /**
     * 
     * @param name
     * @param parent
     * @param child
     * @param axis use [0,0,1] if not needed, e.g. not needed for jointType = fixed
     * @param origin The relationship between the two links, before any motion is applied
     * @param jointType
     */
    public Joint(String name, Link parent, Link child, 
                 Vector3d axis, RPYXYZ origin, JointType jointType){
        this.name = name;
        this.parent = parent;
        this.child = child;
        this.axis = axis;
        this.origin = origin;
        this.jointType = jointType;
        //child.setParent(this);
    }
    public Joint(String name, Link parent, Link child, 
                RPYXYZ rpyxyz, JointType jointType){
        this.name = name;
        this.parent = parent;
        this.child = child;
        this.axis = new Vector3d();
        this.origin = rpyxyz;
        this.jointType = jointType;
    }
    public Joint(String name, Link parent, Link child, JointType jointType){
        this.name = name;
        this.parent = parent;
        this.child = child;
        this.axis = new Vector3d();
        this.origin = new RPYXYZ(new RPY(0d,0d,0d), new Vector3d());
        this.jointType = jointType;
    }
    
    public String getName(){
        return name;
    }
    
    public Link getChild(){
        return child;
    }
    public Link getParent(){
        return parent;
    }
    public RPYXYZ getRPYXYZ(){
        return origin;
    }
    public Vector3d getAxis(){
        return axis;
    }
   
    public String toXML(){
        StringBuilder sb = new StringBuilder();
        sb.append("<joint name=\"");
        sb.append(getName());
        sb.append("\" type=\"");
        sb.append(jointType);
        sb.append("\">\n");
        if (parent != null){
            sb.append("\t<parent link=\"");
            sb.append(parent.getName());
            sb.append("\"/>\n");
        }
        if (child != null){
            sb.append("\t<child link=\"");
            sb.append(child.getName());
            sb.append("\"/>\n");
        }
        sb.append("\t");
        sb.append(Chain.toXML(origin));
        if (axis != null){
            sb.append("\t<axis xyz=\"");
            sb.append(String.valueOf(axis.x));
            sb.append(" ");
            sb.append(String.valueOf(axis.y));
            sb.append(" ");
            sb.append(String.valueOf(axis.z));
            sb.append("\"/>\n");
        }
        sb.append("    </joint>\n");
        return sb.toString();
    }
    
    /**
     * Get the joint axisInParentFrame in the parent frame.
     * 
     * @see def get_joint_dh_params(self, rel_link_frame, axisInParentFrame):
     */
    public Vector3d getAxisInParentFrame(Matrix4d parentFrame){
        // parent_tf_to_child_tf = kh.get_extrinsic_rotation(joint_data['rpy'])
        Matrix3d parentTF2ChildTF = RotationUtils.getExtrinsicRotation(origin.rpy());
        // axis_in_parent_tf = np.matmul(parent_tf_to_child_tf, joint_data['axisInParentFrame'])
        Vector3d axisInParentFrame = new Vector3d(axis);
        parentTF2ChildTF.transform(axisInParentFrame);
        return axisInParentFrame;
    }
    /*
        # for joint_name, joint_data in self.urdf_joints.items():
        #     print(joint_name)
        #     parent_tf_to_child_tf = kh.get_extrinsic_rotation(joint_data['rpy'])
        #     # print(parent_tf_to_child_tf)

        #     axis_in_parent_tf = np.matmul(parent_tf_to_child_tf, joint_data['axisInParentFrame'])
        #     self.publish_arrow(joint_data['parent'], joint_data['xyz'], axis_in_parent_tf)
        #     # print(axis_in_parent_tf)
    */
}
