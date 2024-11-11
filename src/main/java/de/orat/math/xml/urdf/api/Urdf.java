package de.orat.math.xml.urdf.api;

import de.orat.math.xml.urdf.api.Chain.DH;
import de.orat.math.xml.urdf.api.Chain.RPY;
import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.api.Chain.mDH;
import de.orat.math.xml.urdf.api.InertialParameters.InertialMatrixElements;
import de.orat.math.xml.urdf.api.Joint.JointType;
import de.orat.math.xml.urdf.visual.Cylinder;
import de.orat.math.xml.urdf.visual.Shape;
import de.orat.math.xml.urdf.util.RotationUtils;
import de.orat.math.xml.urdf.visual.Box;
import de.orat.math.xml.urdf.visual.Box.Size;
import de.orat.math.xml.urdf.visual.Material;
import de.orat.math.xml.urdf.visual.Mesh;
import de.orat.math.xml.urdf.visual.Sphere;
import java.awt.Color;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.jogamp.vecmath.AxisAngle4d;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Vector3d;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

// https://answers.ros.org/question/52535/parsing-a-urdf-model-in-rosjava/

// online urdf visualizer
// https://mymodelrobot.appspot.com/

// http://sachinchitta.github.io/urdf2/
// https://github.com/ros/urdf/issues/13

public class Urdf {
    
    private Document doc;
    
    public Urdf(InputStream urdf) throws SAXException, IOException, ParserConfigurationException{
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        DocumentBuilder dBuilder = factory.newDocumentBuilder();
        doc = dBuilder.parse(urdf);
    }

    public Chain createChain(){
        doc.getDocumentElement().normalize();

        // robot
        Element robotElement = doc.getDocumentElement();
        String robotName = robotElement.getNodeName();
        System.out.println("Root element: " + robotName);
        Chain robot = new Chain(robotName);

        
        // get links
        
        NodeList linkTags = robotElement.getElementsByTagName("link");
        for(int i = 0; i < linkTags.getLength(); i++){
            Node nNode = linkTags.item(i);
            Element elem = (Element) nNode;
            String linkName = elem.getAttribute("name");
            //System.out.println("Current Element: type=" + nNode.getNodeName()+", name="+linkName);
            Link link = new Link(linkName);
            robot.addLink(link);
            
            
            // get link child elements
            
            // Note: multiple instances of <visual> tags can exist for the same link. 
            // The union of the geometry they define forms the collision representation of the link. 
            List<Element> visualChildElements = getDirectChildsByTag(elem, "visual");
            for (Element visual: visualChildElements){
               RPYXYZ origin = parseOrigin(visual);
               VisualParameters visu = new VisualParameters(origin, parseGeometry(visual));
               // optional name attribute
               String name = visual.getAttribute("name");
               if (!name.isEmpty()) visu.setName(name);
               // optional material child element
               Material material = parseMaterial(visual);
               if (material != null){
                   visu.set(material);
               }
               link.add(visu);
            }
            
            // optional, more than one is allowed
            // Note: multiple instances of <collision> tags can exist for the same link. 
            // The union of the geometry they define forms the collision representation of the link. 
            List<Element> collisionChildElements = getDirectChildsByTag(elem, "collision");
            for (Element collision: collisionChildElements){
               RPYXYZ origin = parseOrigin(collision);
               CollisionParameters coll = new CollisionParameters(origin, parseGeometry(collision));
               // optional name attribute
               String name = collision.getAttribute("name");
               if (!name.isEmpty()) coll.setName(name);
               link.add(coll);
            }
            
            // (optional: defaults to a zero mass and zero inertia if not specified)
            Element inertial = getDirectChildByTag(elem, "inertial");
                if (inertial != null){
                InertialParameters iner = new InertialParameters(parseOrigin(inertial), 
                        parseMass(inertial), parseInertia(inertial));
                link.set(iner);
            }
        }

        
        // get joints
        
        //NodeList jointTags = robotElement.getElementsByTagName("joint");
        List<Element> joints = getDirectChildsByTag(robotElement, "joint");
        for (int j = 0; j < joints.size(); j++) {
            Element joint = joints.get(j);
            String elementTypeName = joint.getNodeName();
            
            String jointName = joint.getAttribute("name");
            JointType jointType = parseJointType(joint);
            System.out.println("Current Element: type="+ elementTypeName+", name="+jointName);

             
            // get joint child elements
            
            List<Element> parentLinks = getDirectChildsByTag(joint, "parent");
            String parentLinkName = parentLinks.iterator().next().getAttribute("link");

            List<Element> childLinks = getDirectChildsByTag(joint, "child");
            String childLinkName = childLinks.iterator().next().getAttribute("link");

            RPYXYZ origin = parseOrigin(joint);
            
            List<Element> axisElements = getDirectChildsByTag(joint, "axis");
            Element axisElement = null;
            Iterator<Element> it = axisElements.iterator();
            if (it.hasNext()){
                axisElement = it.next();
            }

            Vector3d axis;
            if (axisElement != null){
                axis = parseAxis(axisElement);
            } else {
                // kommt vor, wenn parent currentLink das root currentLink ist
                axis = new Vector3d(0,0,1);
            }
            
            Link parentLink = robot.getLink(parentLinkName);
            Link childLink = robot.getLink(childLinkName);
            
            Joint jointObj = new Joint(jointName, parentLink, 
                    childLink, axis, origin, jointType);

            // set the joint into the adjacent links
            robot.addJoint(jointObj);
        }
        robot.init();
        //robot.findRootLink();
        //robot.calcTFs();
        return robot;
    }
    
    private static RPYXYZ parseOrigin(Element parentElement){
        List<Element> originElements = getDirectChildsByTag(parentElement, "origin");
        if (originElements.isEmpty()){
           return new RPYXYZ(new RPY(0,0,0), new Vector3d());
        }
        Element originElement = originElements.iterator().next();
        Vector3d position = parsePosition(originElement);
        RPY orientation = parseOrientation(originElement);
        return new RPYXYZ(orientation, position);
    }
    private static Material parseMaterial(Element parentElement){
        Element material = getDirectChildByTag(parentElement, "material");
        if (material == null) return null;
        Material result = new Material(material.getAttribute("name"));
        // optional color
        Color color = parseColor(material);
        if (color != null){
            result.setColor(color);
        }
        //TODO optional texture
        return result;
    }
    
    private static Color parseColor(Element parentElement){
        Element color = getDirectChildByTag(parentElement, "color");
        if (color == null) return null;
        String values = color.getAttribute("rgba");
        String[] strValues = values.split("\\s+");
        return new Color(Float.parseFloat(strValues[0]), 
                Float.parseFloat(strValues[1]), 
                Float.parseFloat(strValues[2]),
               Float.parseFloat(strValues[3]));
    }
    private static InertialMatrixElements parseInertia(Element parentElement){
        // <inertia ixx="0.00443333156" ixy="0.0" ixz="0.0" iyy="0.00443333156" 
        //          iyz="0.0" izz="0.0072"/>
        Element inertia = getDirectChildByTag(parentElement, "inertia");  
        return new InertialMatrixElements(
                Float.parseFloat(inertia.getAttribute("ixx")), 
                Float.parseFloat(inertia.getAttribute("ixy")), 
                Float.parseFloat(inertia.getAttribute("ixz")), 
                Float.parseFloat(inertia.getAttribute("iyy")), 
                Float.parseFloat(inertia.getAttribute("iyz")), 
                Float.parseFloat(inertia.getAttribute("izz")));
    }
    
    private static double parseMass(Element parentElement){
        Element mass = getDirectChildByTag(parentElement, "mass");
        String value = mass.getAttribute("value");
        return Float.parseFloat(value);
    }
    
    private static Shape parseGeometry(Element parentElement){
        Shape result = null;
        List<Element> geometryElements = getDirectChildsByTag(parentElement, "geometry");
        Element el = geometryElements.iterator().next();
        Element element = getDirectChildByTag(el, "box");
        if (element != null){
            String size = element.getAttribute("size");
            String[] strValues = size.split("\\s+");
            return new Box(new Size(Double.parseDouble(strValues[0]), 
                Double.parseDouble(strValues[1]), 
                Double.parseDouble(strValues[2])));
        } else {
            element = getDirectChildByTag(el, "cylinder");
            if (element != null){
                double length = Double.parseDouble(element.getAttribute("length"));
                double radius = Double.parseDouble(element.getAttribute("radius"));
                return new Cylinder(length, radius);
            } else {
                element = getDirectChildByTag(el, "sphere");
                if (element != null){
                    double radius = Double.parseDouble(element.getAttribute("radius"));
                    return new Sphere(radius);
                } else {
                    element = getDirectChildByTag(el, "mesh");
                    if (element != null){
                        String fileName = element.getAttribute("filename");
                        Mesh mesh = new Mesh(fileName);
                        String scaleString = element.getAttribute("scale");
                        if (!scaleString.isEmpty()){
                            mesh.setScale(Double.parseDouble(scaleString));
                        }
                        return mesh;
                    }
                }
            }
        } 
        return result;
    }
    
    private static JointType parseJointType(Element elem){
        String jointTypeName = elem.getAttribute("type");
        return JointType.valueOf(jointTypeName);
    }
    
    private static Vector3d parsePosition(Element elem){
        String positions = elem.getAttribute("xyz");
        String[] strValues = positions.split("\\s+");
        return new Vector3d(Double.parseDouble(strValues[0]), 
                Double.parseDouble(strValues[1]), 
                Double.parseDouble(strValues[2]));
    }
    
    private static RPY parseOrientation(Element elem){
        String orientation = elem.getAttribute("rpy");
        String[] strValues = orientation.split("\\s+");
        return new RPY(Double.parseDouble(strValues[0]), 
                Double.parseDouble(strValues[1]), 
                Double.parseDouble(strValues[2]));
    }
    
    private static Vector3d parseAxis(Element elem){
        String axis = elem.getAttribute("xyz");
        String[] strValues = axis.split("\\s+");
        return new Vector3d(Double.parseDouble(strValues[0]), 
                Double.parseDouble(strValues[1]), 
                Double.parseDouble(strValues[2]));
    }
    
    private static List<Element> getDirectChildsByTag(Element el, String sTagName) {
        NodeList allChilds = el.getElementsByTagName(sTagName);
        List<Element> res = new ArrayList<>();
        for (int i = 0; i < allChilds.getLength(); i++) {
            if (allChilds.item(i).getParentNode().equals(el))
                res.add((Element) allChilds.item(i));
        }
        return res;
    }
    private static Element getDirectChildByTag(Element el, String sTagName) {
        NodeList allChilds = el.getElementsByTagName(sTagName);
        for (int i = 0; i < allChilds.getLength(); i++) {
            if (allChilds.item(i).getParentNode().equals(el))
                return (Element) allChilds.item(i);
        }
        return null;
    }
    
    
    // https://adohaha.github.io/DH2URDF/
    
     /**
     * Generally as the DH notation translates into series of transformations 
     * Rot(z,theta)Trans(z,d)Trans(x,a)Rot(x,alpha) two joints -- active and 
     * fixed are needed for each row of the table.
     * 
     * DH parameters are a kinematic description and not a structural one. There 
     * are an infinite number of robot structures that could satisfy some set of 
     * DH parameters, and this code just outputs one. The output may not be the 
     * structure you had in mind when you wrote down the DH parameters.<p>
     * 
     * UR5e nominal parameters set:<p>
     * 
     * |th|d|a|alpha|R|
     * -------
     * |0|0.1625|0|pi/2|true|
     * |0|0|-0.425|0|true|
     * |0|0|0.3922|0|true|
     * |0|0.1333|0|pi/2|true|
     * |0|0.0997|0|-pi/2|true|
     * |0|0.0996|0|0|true|
     * 
     * Erster Aufruf mit der 2. DH parameter Zeile!!!
     * TODO warum das denn?
     * 
     * @param dh
     * @param add_x
     * @param revolute
     * @return 
     * 
     * 
     */
    //public static void addDHtoURDFChain(DH dh, int row_no) {
        /* function gets a dictionary representing row in DH table, returns xml node based on two links
        additional code representing x axis can be added if add_x is true
        */
        /*var alpha, d, row_no, row_xml, template_xml;
        row_no = row_dict.row_no;
        row_dict.name = row_no;
        row_dict.previous_name = row_no - 1;
        if (row_dict.R) {
          row_dict.type = "revolute";
        } else {
          row_dict.type = "prismatic";
        }
        template_xml = this.row_template_insert;
        if (add_x) {
          template_xml = template_xml + this.row_template_add_x;
        }*/
        
        /*Robot_Maker.prototype.row_template_insert = 
                  "<link name='currentLink{{name}}'></link> "
                + "<link name='currentLink{{name}}_x_axis'>"
                +   "<visual>"
                +       "<origin xyz='0 0 0.25' rpy='0 0 0'/> "
                +       "<material name='red' />"
                +       "<geometry>"
                +           "<cylinder length='0.5' radius='0.05'/>"
                +       "</geometry>"
                + "  </visual>"
                + "</link> "
                + "<joint name='q{{name}}' type='{{type}}'> "
                +   "<origin xyz='0 0 {{d}}' rpy='0 0 {{th}}'/> "
                +   "<parent currentLink='currentLink{{previous_name}}_passive' /> "
                +   "<child currentLink='currentLink{{name}}' /> "
                +   "<axis xyz='0 0 1'/> "
                + "</joint> "
                + "<joint name='q{{row_no}}_passive' type='fixed'> "
                +   "<origin xyz='{{a}} 0 0' rpy='{{alpha}} 0 0'/> "
                +   "<parent currentLink='currentLink{{name}}' /> "
                +   "<child currentLink='currentLink{{name}}_passive' /> "
                + "</joint>";

        */
        // There is the option of using modified Denavit-Hartenberg parameters, 
        // where the sequence of transformations is different. 
        // Actually, this gives the opportunity toOld use only one currentLink per row 
        // (while treating each row equally),
        //  but some math is needed toOld calculate the start translation between joints. 
        // That is we translation=[a;0;0]+ R[x,alpha] [0;0;d]
        /*if (row_dict.modified_dh) {
          alpha = parseFloat(row_dict.alpha);
          d = parseFloat(row_dict.d);
          row_dict.dy = -Math.sin(alpha) * d;
          row_dict.dz = Math.cos(alpha) * d;
          console.log(row_dict.dy);
          console.log(row_dict.dz);
          template_xml = this.modified_dh_row_template_insert;
        }
        row_xml = Mustache.render(template_xml, row_dict);
        return row_xml;
      }
*/
        
    //}
    
    /**
     * Mdh gives the opportunity to use only one link per row (while treating 
     * each row equally), but some math is needed to calculate the start 
     * translation between joints. 
     * 
     * That is we translation=[a;0;0]+ R[x,alpha] [0;0;d]<p>
     *
     * @param chain the chain where the created currentLink and joint objects are added
     * @param parentLink
     * @param mdh
     * @param linkName
     * @param revolute 
     * 
     * TODO
     * vermutlich funktioniert das bereits so und ich muss das nur noch testen!
     */
    public static void addMDHtoURDFChain(Chain chain, Link parentLink, mDH mdh, String linkName) {
        double a = mdh.r();
        double theta = mdh.theta();
        double alpha = mdh.alpha();
        double d = mdh.d();
        double dy = -Math.sin(alpha) * d;
        double dz = Math.cos(alpha) * d;
           
        Link link = new Link(linkName);
        Joint joint = new Joint(parentLink.getName()+"_"+linkName, parentLink, link,  
                 new Vector3d(0d,0d,1d), new RPYXYZ(new RPY(alpha, 0d, theta), new Vector3d(a, dy, dz)), JointType.revolute);
        chain.addLink(link);
        chain.addJoint(joint);
    }
     
    
    // https://gitlab.com/michaelryangreer/Instructional/-/blob/main/URDF/DH_to_URDF.py
    // Der obige Code erzeugt fake-links zur Visualisierung auch der joints

    //def xml_string(DH_Params, scale=1):
    /**
     * Create a chain, based on a sequence of dh-parameters. 
     * 
     * For each line of Denavit-Hartenberg parameters two links are created: an 
     * acutator link, attached by a fixed joint to the previous link with 
     * transformation R_z(t) and a second link attached by a revolute joint to 
     * the actuator link.<p>
     * 
     * TODO<br>
     * - add inertial parameters, friction parameter (static, viscous)<br>
     * - ziemlich genau nachgebaut, scheint aber immer noch nicht zu stimmen<p>
     * 
     * @param name
     * @param dhParams 
     * @param jointNames 
     * @param visualElements if true, additional collision parameters with shapes are created
     * @return chain representing the kinematic and dynamic structure corresponding to the given dh-parameters.
     */
    public static Chain toChainOld(String name, DH[] dhParams, String[] jointNames, 
                                boolean visualElements/*, boolean xaxes*/){

        Chain chain = new Chain(name);

        // transforms = joint_transforms(DH_Params)
        // corresponds to technical linkFrames for the links, relativ to the parent currentLink
        // +1 corresponding to the count of dhParams lines
        // dh-parameter based transformations
        Matrix4d[] transforms = to(dhParams);

        //frames = joint_frames(transforms)
        // multiplicated adjacent dhtransformation
        // dh-paramter based joint frames 
        Matrix4d[] frames = toJointTFs(transforms);

        Link parent = null;
        //for i in range(len(transforms) - 1):
        for (int i=0;i<transforms.length-1;i++){

            Matrix4d transform = transforms[i]; // joint transformation == parent-link-TF?
            Matrix4d jointFrame = frames[i]; // joint frame == child-link-TF
            
            // fake currentLink zur Visualisierung des Joint als currentLink
            // or actuator as fixed currentLink with inertia

            //rpy = R.from_matrix(jointFrame[0:3,0:3]).as_euler('XYZ')
            Matrix3d rot = new Matrix3d();
            jointFrame.get(rot);
            RPY rpy = RotationUtils.toRPY(rot, false);
            Vector3d t = new Vector3d();
            transform.get(t);
            RPYXYZ rpyxyz = new RPYXYZ(rpy, t);
            //InertialParameters inertialParameters = new InertialParameters(rpyxyz);
            // The joint is located at the origin of the child link.
            // The actuator-link represents the dh-joint that's why it frame corresponds
            // to jointFrame
            // So intertial parameters are only well defined if nominal dh-parameters
            // are used because calibrated dh-parameters can move d along the z-axis a lot
            Link actuatorLink = new Link("alink_"+String.valueOf(i));
            //actuatorLink.set(inertialParameters);
            if (visualElements){
                actuatorLink.add(new VisualParameters(rpyxyz, new Cylinder(0.05d, 0.01d)));
            }
            chain.addLink(actuatorLink);
            
            
            // If not on the first transformation, fix the cylinder to the previous currentLink
            if (i != 0){ // parent != null
                // The "fixed-joint" has no transformation (origin) because the transformation from the
                // parent-link to the child-link (representing the "revolute-joint"
                // actuator-base-part) is defined in the child-link 
                Joint actuatorJoint = new Joint(parent.getName()+"_to_"+actuatorLink.getName(), 
                        parent, actuatorLink, JointType.fixed);
                chain.addJoint(actuatorJoint);
                //TODO
                // warum wird hier kein rpyxyz eingetragen?
            }
            
            
            // get next link data
            
            // Add a currentLink/cylinder that goes from the current origin to the next one
            //origins_vector = transforms[i + 1][0:3,3]
            Vector3d origins_vector = new Vector3d();
            transforms[i+1].get(origins_vector);
            //origins_vector_norm = np.linalg.norm(origins_vector)
            double origins_vector_norm = origins_vector.length();
            // cylinder_origin is the half of the translation vector between the
            // two joints
            Vector3d cylinder_origin = new Vector3d(origins_vector);
            cylinder_origin.scale(0.5d);
            
            Link currentLink;
            //if (origins_vector_norm != 0.0):
            // also sich nicht kreuzende joint axes (nicht schneidende)
            if (origins_vector_norm != 0d){

                //origins_vector_unit = origins_vector/origins_vector_norm
                Vector3d origins_vector_unit = new Vector3d(origins_vector);
                origins_vector_unit.normalize();
                //axis = np.cross(origins_vector, np.array([0, 0, -1]))
                Vector3d axis = new Vector3d();
                //FIXME warum das?
                axis.cross(origins_vector, new Vector3d(0,0,-1));
                //axis_norm = np.linalg.norm(axis)
                double axis_norm = axis.length();
                //if (axis_norm != 0.0):
                if (axis_norm != 0d){
                    //axis = axis/np.linalg.norm(axis)
                    axis.normalize();
                }
                //angle = np.arccos(origins_vector_unit @ np.array([0, 0, 1]))
                double angle = Math.acos(origins_vector_unit.dot(new Vector3d(0,0,1)));
                
                //rpy = R.from_rotvec(angle * axis).as_euler('XYZ')
                Matrix3d m = new Matrix3d();
                m.set(new AxisAngle4d(axis, angle));
                rpy = RotationUtils.toRPY(m, false);
                rpyxyz = new RPYXYZ(rpy, cylinder_origin);
                
                currentLink = new Link("link_"+String.valueOf(i));
                
                //InertialParameters inertialParameters1 = new InertialParameters(rpyxyz);
                //currentLink.set(inertialParameters1);
                if (visualElements){
                    VisualParameters visualParameters = new VisualParameters(rpyxyz, new Cylinder(0.05d, 0.01d));
                    //visualParameters.set(new Material("red")); // Farbe muss vorher definiert werden
                    currentLink.add(visualParameters);
                }
                /*outstring = outstring + "\t<link name='l{}'>\n".format(i)
                outstring = outstring + "\t\t<visual>\n"
                outstring = outstring + "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(rpy[0], rpy[1], rpy[2], cylinder_origin[0], cylinder_origin[1], cylinder_origin[2])
                outstring = outstring + "\t\t\t<geometry>\n"
                outstring = outstring + "\t\t\t\t<cylinder length='{}' radius='0.4'/>\n".format(origins_vector_norm) 
                outstring = outstring + "\t\t\t</geometry>\n"
                outstring = outstring + "\t\t\t<material name='red'/>\n"
                outstring = outstring + "\t\t</visual>\n"
                outstring = outstring + "\t</link>\n"
                */
            } else {
                // wenn sich aber die aufeindernfolgenden links schneiden, dann kann
                // ich zwar kein currentLink visualisieren, brauche das currentLink aber trotzdem
                // hier muss noch die Transformation mit gespeichert werden, später für die inertials
                // und den nachfolgenden joint
                //TODO
                currentLink = new Link("alink_"+String.valueOf(i));
            } 
            
            chain.addLink(currentLink);

            // Add the actual joint between the cylinder and currentLink

            //FIXME
            // warum ist hier rpy==0? vermutlich wegen dem actuator/fixed-link
            
            /*outstring = outstring + "\t<joint name='move_l{}_from_a{}' type='{}'>\n".format(i, i, jointType)
            outstring = outstring + "\t\t<parent currentLink='a{}'/>\n".format(i)
            outstring = outstring + "\t\t<child currentLink='l{}'/>\n".format(i)
            outstring = outstring + "\t\t<axis xyz='{} {} {}'/>\n".format(jointFrame[0,2], jointFrame[1,2], jointFrame[2,2])
            outstring = outstring + "\t\t<origin rpy='0 0 0' xyz='{} {} {}'/>\n".format(transform[0,3], transform[1,3], transform[2,3])   
            outstring = outstring + "\t</joint>\n" 
            */
            Vector3d axis = new Vector3d(jointFrame.m02, jointFrame.m12, jointFrame.m22);
            //FIXME
            // hier sollte für alink_0 (0,0,0.1625) herauskommen - ist aber 0,0,0
            // in frames[i+1] steht vermutlich das richtige aber warum ist das so?
            //TODO
            
            Vector3d position = new Vector3d(); //transform.m03, transform.m13, transform.m23);
            transform.get(position);
            String jointName = actuatorLink.getName()+"_to_"+currentLink.getName();
            if (jointNames != null){
                jointName = jointNames[i];
            }
            // The joint is located at the origin of the child link.
            Joint joint = new Joint(jointName, 
                    actuatorLink, currentLink, 
                 axis, new RPYXYZ(new RPY(0d,0d,0d), position), JointType.revolute);
            chain.addJoint(joint);
            parent = currentLink;
        }
        
        //TODO
        // add tool link (mit inertia) und verbinde mit fixed joint und fix rot angel um z 
        // der vorherigen joint
        
        //FIXME
        // doppelt gemoppelt mit transforsm und jointFrames habe ich ddie Matrizen ja teilweise bereits berechnet
        // und sollte sie besser direkt speichern
        chain.init();
        //chain.findRootLink();
        //chain.calcTFs();
        return  chain;
    }

    /**
     * Create joint transformation for each line of dh parameters.
     * 
     * This is the Transformation from the parent link to the child link of a
     * given joint.
     * 
     * @param dhParams
     * @return joint transformations, one more than the number of dh parameter lines 
     * to include a base transformation
     */
    private static Matrix4d[] to(DH[] dhParams){
    //def joint_transforms(DH_Params):

        Matrix4d[] result = new Matrix4d[dhParams.length+1];

        result[0] = new Matrix4d();
        result[0].setIdentity();

        int i=1;
        for (DH dh: dhParams){
            result[i++] = RotationUtils.to(dh);
        }
        return result;
    }

    //def joint_frames(transforms):
    /**
     * Determination of the joints technical frames which is identical to the
     * child links frames.
     * 
     * @param dhTransforms dh-parameters based transformation from parent link to child link
     * @return joint (child-link) technical frames
     */
    private static Matrix4d[] toJointTFs(Matrix4d[] dhTransforms){ 
        Matrix4d[] result = new Matrix4d[dhTransforms.length];
        result[0] = new Matrix4d(dhTransforms[0]);
        for (int i=1;i<dhTransforms.length;i++){
            result[i] = new Matrix4d(result[i-1]);
            result[i].mul(dhTransforms[i]);
        }
        return result;
    }
    

    // new
    // eigentlich sollte hier dafür gesort werden das das erste TF die x-Achse des
    // 2. übernimmt ... Wie geht das?
    private static Matrix4d[] toLinkTFs(Matrix4d[] jointTransformations){ 
        Matrix4d[] result = new Matrix4d[jointTransformations.length+1];
        result[0] = new Matrix4d();
        result[0].setIdentity();
        for (int i=1;i<jointTransformations.length;i++){
            result[i] = new Matrix4d(result[i-1]);
            result[i].mul(jointTransformations[i]);
        }
        return result;
    }

    // neuer Versuch
    // zuerst einmal ohne split eines actuator links
    // für das erste Gelenk für die x-Achse des 2. Gelenks übernommen
    /**
     * 
     * @param name
     * @param dhParams
     * @param jointNames optional naming of the joints, can be null
     * @param visualElements
     * @param xaxes
     * @return 
     */
    public static Chain toChain(String name, DH[] dhParams, String[] jointNames, 
                                boolean visualElements, boolean xaxes){

        Chain chain = new Chain(name);

        Matrix4d[] transforms = to(dhParams);
        Matrix4d[] linkFrames = toLinkTFs(transforms);

        Link parent = null;
        for (int i=0;i<transforms.length;i++){

            Matrix4d transform = transforms[i]; // relative transformation
            Matrix4d linkFrame = linkFrames[i]; // absolute frames
            
            // fake currentLink zur Visualisierung des Joint als currentLink
            // or actuator as fixed currentLink with inertia

            //rpy = R.from_matrix(linkFrame[0:3,0:3]).as_euler('XYZ')
            Matrix3d rot = new Matrix3d();
            linkFrame.get(rot);
            RPY rpy = RotationUtils.toRPY(rot, false);
            RPYXYZ rpyxyz = new RPYXYZ(rpy, new Vector3d(transform.m03, transform.m13, transform.m23));
            //InertialParameters inertialParameters = new InertialParameters(rpyxyz);
            Link actuatorLink = new Link("alink_"+String.valueOf(i));
            //actuatorLink.set(inertialParameters);
            if (visualElements){
                actuatorLink.add(new VisualParameters(rpyxyz, new Cylinder(1d, 0.5d)));
            }
            chain.addLink(actuatorLink);
            //# We need toOld create a cylinder toOld represent the joint
            /*outstring = outstring + "\t<link name='a{}'>\n".format(i)
            outstring = outstring + "\t\t<visual>\n"
            outstring = outstring + "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(rpy[0], rpy[1], rpy[2], transform[0,3], transform[1,3], transform[2,3])
            outstring = outstring + "\t\t\t<geometry>\n"
            outstring = outstring + "\t\t\t\t<cylinder length='1' radius='0.5'/>\n"
            outstring = outstring + "\t\t\t</geometry>\n"
            outstring = outstring + "\t\t\t<material name='blue'/>\n"
            outstring = outstring + "\t\t</visual>\n"
            outstring = outstring + "\t</link>\n"*/
            
            //# And a joint toOld connect it toOld the currentLink
            
            //# If the index is not zero, connect it toOld the previous currentLink
            //# If not on the first transformation, fix the cylinder toOld the previous currentLink
            /*if(i != 0):
                outstring = outstring + "\t<joint name='fix_a{}_to_l{}' type='fixed'>\n".format(i, i-1)
                outstring = outstring + "\t\t<parent currentLink='l{}'/>\n".format(i-1)
                outstring = outstring + "\t\t<child currentLink='a{}'/>\n".format(i)
                outstring = outstring + "\t\t<origin rpy='0 0 0' xyz='0 0 0'/>\n"
                outstring = outstring + "\t</joint>\n"
            */
            if (i != 0){ // parent != null
                Joint actuatorJoint = new Joint(parent.getName()+"_to_"+actuatorLink.getName(), 
                        parent, actuatorLink, JointType.fixed);
                chain.addJoint(actuatorJoint);
            }
            
            
            // get next link data
            
            //# Add a currentLink/cylinder that goes from the current origin toOld the next one
            //origins_vector = transforms[i + 1][0:3,3]
            Vector3d origins_vector = new Vector3d();
            transforms[i+1].get(origins_vector);
            
            //origins_vector_norm = np.linalg.norm(origins_vector)
            double origins_vector_norm = origins_vector.length();
            
            Vector3d cylinder_origin = new Vector3d(origins_vector);
            cylinder_origin.scale(0.5d);
            
            Link currentLink;
            //if (origins_vector_norm != 0.0):
            // also sich nicht kreuzende joint axes
            if (origins_vector_norm != 0d){

                //origins_vector_unit = origins_vector/origins_vector_norm
                Vector3d origins_vector_unit = new Vector3d(origins_vector);
                origins_vector_unit.normalize();

                //axis = np.cross(origins_vector, np.array([0, 0, -1]))
                Vector3d axis = new Vector3d();
                //FIXME warum das?
                axis.cross(origins_vector, new Vector3d(0,0,-1));

                //axis_norm = np.linalg.norm(axis)
                double axis_norm = axis.length();
                
                //if (axis_norm != 0.0):
                if (axis_norm != 0d){
                    //axis = axis/np.linalg.norm(axis)
                    axis.normalize();
                }
                //angle = np.arccos(origins_vector_unit @ np.array([0, 0, 1]))
                double angle = Math.acos(origins_vector_unit.dot(new Vector3d(0,0,1)));
                
                //rpy = R.from_rotvec(angle * axis).as_euler('XYZ')
                Matrix3d m = new Matrix3d();
                m.set(new AxisAngle4d(axis, angle));
                RPY rpy1 = RotationUtils.toRPY(m, false);
                RPYXYZ rpyxyz1 = new RPYXYZ(rpy1, cylinder_origin);
                
                currentLink = new Link("link_"+String.valueOf(i));
                
                //InertialParameters inertialParameters1 = new InertialParameters(rpyxyz1);
                //currentLink.set(inertialParameters1);
                if (visualElements){
                    currentLink.add(new VisualParameters(rpyxyz, new Cylinder(1d, 0.5d)));
                }
                /*outstring = outstring + "\t<link name='l{}'>\n".format(i)
                outstring = outstring + "\t\t<visual>\n"
                outstring = outstring + "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(rpy[0], rpy[1], rpy[2], cylinder_origin[0], cylinder_origin[1], cylinder_origin[2])
                outstring = outstring + "\t\t\t<geometry>\n"
                outstring = outstring + "\t\t\t\t<cylinder length='{}' radius='0.4'/>\n".format(origins_vector_norm) 
                outstring = outstring + "\t\t\t</geometry>\n"
                outstring = outstring + "\t\t\t<material name='red'/>\n"
                outstring = outstring + "\t\t</visual>\n"
                outstring = outstring + "\t</link>\n"
                */
            } else {
                // wenn sich aber die aufeindernfolgenden links schneiden, dann kann
                // ich zwar kein currentLink visualisieren, brauche das currentLink aber trotzdem
                // hier muss noch die Transformation mit gespeichert werden, später für die inertials
                // und den nachfolgenden joint
                //TODO
                currentLink = new Link("alink_"+String.valueOf(i));
            } 
            
            chain.addLink(currentLink);

            // # Add the actual joint between the cylinder and currentLink

            //FIXME
            // warum ist hier rpy==0? vermutlich wegen dem actuator/fixed-link
            
            /*outstring = outstring + "\t<joint name='move_l{}_from_a{}' type='{}'>\n".format(i, i, jointType)
            outstring = outstring + "\t\t<parent currentLink='a{}'/>\n".format(i)
            outstring = outstring + "\t\t<child currentLink='l{}'/>\n".format(i)
            outstring = outstring + "\t\t<axis xyz='{} {} {}'/>\n".format(linkFrame[0,2], linkFrame[1,2], linkFrame[2,2])
            outstring = outstring + "\t\t<origin rpy='0 0 0' xyz='{} {} {}'/>\n".format(transform[0,3], transform[1,3], transform[2,3])   
            outstring = outstring + "\t</joint>\n" 
            */
            Vector3d axis = new Vector3d(linkFrame.m02, linkFrame.m12, linkFrame.m22);
            //FIXME
            // hier sollte für alink_0 (0,0,0.1625) herauskommen - ist aber 0,0,0
            // in linkFrame[i+1] steht vermutlich das richtige aber warum ist das so?
            //TODO
            
            Vector3d position = new Vector3d(transform.m03, transform.m13, transform.m23);
            String jointName = actuatorLink.getName()+"_to_"+currentLink.getName();
            if (jointNames != null){
                jointName = jointNames[i];
            }
            Joint joint = new Joint(jointName, 
                    actuatorLink, currentLink, 
                 axis, new RPYXYZ(new RPY(0d,0d,0d), position), JointType.revolute);
            chain.addJoint(joint);
            actuatorLink.addChild(joint);
            currentLink.setParent(joint); // TODO sollte bereits beim Erzeugen des Joint gesetzt werden geht aber nicht, dafür in addJoint();
            parent = currentLink;
        }
        
        //TODO
        // add tool link (mit inertia) und verbinde mit fixed joint und fix rot angel um z 
        // der vorherigen joint
        chain.init();
        //chain.findRootLink();
        //chain.calcTFs();
        return  chain;
    }
     
    // https://github.com/robotology/idyntree-yarp-tools/blob/main/src/modules/urdf2dh/urdf2dh.cpp
    // c++ code urdf toOld dh
    // TODO
    // have a look at the above code
}