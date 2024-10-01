package de.orat.math.xml.urdf.api;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import org.jogamp.vecmath.Matrix4d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Link {
    
    private final String name;
    private Joint parent;
    private List<Joint> children = new ArrayList<>();
    
    // all coordinates defined in <origin>-tags are defined relative to the 
    // link-frame == parent-joint-frame
    private InertialParameters inertialParameters;
    private List<VisualParameters> visualParameters = new ArrayList<>();
    private List<CollisionParameters> collisionParameters = new ArrayList<>();    
    
    
    // defined by the proximal/parent joint by rpy-xyz
    // The transformation from the parent link to this link. 
    Matrix4d relTF;
    
    // relTFs in der chain multiplied
    // TF from current link frame to world frame
    Matrix4d absTF;
    
    //Matrix4d dhTF;
    
    //DH tf from current link frame to world frame
    Matrix4d absDHTF;
    
    public Link(String name){
        this.name = name;
    }
    
    public String getName(){
        return name;
    }
    
    public void set(InertialParameters inertialParameters){
        this.inertialParameters = inertialParameters;
    }
    public void add(VisualParameters visu){
        visualParameters.add(visu);
    }
    public void add(CollisionParameters collision){
        collisionParameters.add(collision);
    }
    public List<VisualParameters> getVisualParameters(){
        return visualParameters;
    }
    public InertialParameters getInertialParameters(){
        return inertialParameters;
    }
    
    void setRelTF(Matrix4d relTF){
        this.relTF = relTF;
    }
    Matrix4d getRelTF(){
        Matrix4d result = relTF; 
        // wenn nicht beides gleichzeitig null ist, dann is das ein Fehler?
        if (parent == null){
            //if (result != null) throw new RuntimeException("")
            if (result == null){
                result = new Matrix4d();
                result.setIdentity();
                relTF = result;
            }
        }
        return result;
    }
    
    void setAbsTF(Matrix4d absTF){
        this.absTF = absTF;
    }
    public Matrix4d getAbsTF(){
        return absTF;
    }
    
    void setAbsDHTF(Matrix4d absDHTF){
        this.absDHTF = absDHTF;
    }
    Matrix4d getAbsDHTF(){
        Matrix4d result = absDHTF;
        if (parent == null){
            if (result == null){
                result = new Matrix4d();
                result.setIdentity();
                absDHTF = result;
            }
        }
        return result;
    }
    /*void setDHTF(Matrix4d dhTF){
        this.dhTF = dhTF;
    }*/
    
    
    void addChild(Joint joint){
        if (joint == null) throw new IllegalArgumentException("try to add joint==null!");
        if (children.contains(joint)) throw new IllegalArgumentException("try to add the included joint \""+joint.getName()+"!");
        children.add(joint);
    }
    void setParent(Joint joint){
        this.parent = joint;
    }
    public Joint getParent(){
        return parent;
    }
    public Link getParentLink(){
        if (parent != null){
            return parent.getParent();
        }
        return null;
    }
    public List<Joint> getChildren(){
        return children;
    }
    public Collection<Link> getChildrenLinks(){
        Collection<Link> result = new ArrayList<>();
        for (Joint joint: getChildren()){
            Link link = joint.getChild();
            if (link != null){
                result.add(link);
            }
        }
        return result;
    }
    
    public String toXML(){
        StringBuilder sb = new StringBuilder();
        sb.append("  <link name=\"");
        sb.append(getName());
        sb.append("\">\n");
        if (inertialParameters != null){
            sb.append(inertialParameters.toXML());
        }
        for (VisualParameters visuals: visualParameters){
            sb.append(visuals.toXML());
        }
        for (CollisionParameters collisions: collisionParameters){
            sb.append(collisions.toXML());
        }
        return sb.toString();
    }
}
