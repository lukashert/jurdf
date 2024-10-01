package de.orat.math.xml.urdf;

import de.orat.math.xml.urdf.api.Chain;
import de.orat.math.xml.urdf.api.Chain.RPYXYZ;
import de.orat.math.xml.urdf.api.InertialParameters;
import de.orat.math.xml.urdf.api.Joint;
import de.orat.math.xml.urdf.api.Link;
import de.orat.math.xml.urdf.api.Urdf;
import de.orat.math.xml.urdf.util.RotationUtils;
import de.orat.math.xml.urdf.visual.Cylinder;
import de.orat.math.xml.urdf.visual.Shape;
import de.orat.math.xml.urdf.visual.Shape.ShapeType;
import de.orat.math.xml.urdf.api.VisualParameters;
import de.orat.math.xml.urdf.visual.Box;
import de.orat.math.xml.urdf.visual.Box.Size;
import de.orat.math.xml.urdf.visual.Mesh;
import de.orat.math.xml.urdf.visual.Sphere;
import de.orat.view3d.euclid3dviewapi.api.ViewerService;
import de.orat.view3d.euclid3dviewapi.spi.iEuclidViewer3D;
import java.awt.Color;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Visualizer {
    
    private iEuclidViewer3D viewer;
    
    public static void main(String[] args) throws Exception{
        Visualizer visualizer = new Visualizer();
        
        if (visualizer.viewer != null){
            Urdf jurdf = new Urdf(visualizer.getClass().getResourceAsStream("ur5eA.urdf"));
            Chain chain = jurdf.createChain();
            
            visualizer.viewer.open();
            visualizer.add(chain);
          
        } else {
            System.out.println("No Visualizer implementation found!");
        }
    }
    public Visualizer() throws Exception{
        ViewerService viewerService = ViewerService.getInstance();
        viewer = viewerService.getViewer().get();
    }
    
    public void add(Chain chain){
        addRecursive(chain.getRootLink());
    }
    
    private void addRecursive(Link link){
        add(link);
        for (Joint joint: link.getChildren()){
            add(joint);
            addRecursive(joint.getChild());
        }
    }
    
    // sphere in gray
    public void add(Joint joint){
        //System.out.println("addFrame joint \""+joint.getName()+"\" to view.");
        addFrame(joint.getName(), joint.getRPYXYZ(), joint.getChild().getAbsTF(), true, Color.gray);
    }
    // sphere in black
    public void add(Link link){
        System.out.println("add link \""+link.getName()+"\" to view.");
        // visual parameters
        for (VisualParameters visu: link.getVisualParameters()){
            RPYXYZ pose = visu.getRPYXYZ();
            System.out.println("add link frame to view...");
            addFrame(link.getName(), pose, link.getAbsTF(), true, Color.BLACK);
            Shape shape = visu.getShape();
            ShapeType type = shape.getShapeType();
            if (null == type){
                System.out.println("Unknown shape type: \""+type.toString()+"\"!");
            } else switch (type) {
                case cylinder:
                    Cylinder cylinder = (Cylinder) shape; // Ausrichtung in z-Richtung
                    Point3d[] points = getPoints(link.getAbsTF(), pose, cylinder.getLength());
                    viewer.addLine(points[0], points[1], visu.getColor(),
                            cylinder.getRadius(), cylinder.getName());
                    break;
                case sphere:
                    Sphere sphere = (Sphere) shape;
                    Vector3d location = new Vector3d(pose.xyz());
                    link.getAbsTF().get(location);
                    viewer.addSphere(new Point3d(location), sphere.getRadius(), visu.getColor(), sphere.getName(), false);
                    break;
                case box:
                    Box box = (Box) shape;
                    addBox(pose, link.getAbsTF(), box.getSize(), visu.getColor(), box.getName());
                    break;
                    
                case mesh:
                    Mesh mesh = (Mesh) shape;
                    addMesh(mesh.getPath(), link.getAbsTF());
                    break;
                default:
                    System.out.println("Unknown shape type: \""+type.toString()+"\"!");
                    break;
            }
        }
        //TODO
        // coordinate systems für inertial und collision visualisieren
        InertialParameters inertialParameters = link.getInertialParameters();
        if (inertialParameters != null){
            // frame in the center of mass, sphere with color green
            addFrame("", inertialParameters.getRPYXYZ(), link.getAbsTF(), true, Color.GREEN);
        }
    }
    
    /**
     * Add visualisiation of a frame.
     * 
     * @param name name of the frame
     * @param pose in relative coordinates
     * @param absTF transformation to absolute coordinates
     * @param withOrigin if true, than the origin of the frame is visualized by a sphere and the name is shown
     */
    private void addFrame(String name, RPYXYZ pose, Matrix4d absTF, boolean withOrigin, Color color){
        Matrix3d frame = RotationUtils.getExtrinsicRotation(pose.rpy());
        Vector3d trans = pose.xyz();
        double length = 0.03d;
        double radius = 0.005d;
        
        // o
        Point3d p1 = new Point3d(trans);
        absTF.transform(p1);
        if (withOrigin){
            viewer.addSphere(p1, 2d*radius, color, name, false);
        }
        // x
        Point3d p2 = new Point3d(frame.m00, frame.m10, frame.m20);
        p2.scale(length);
        p2.add(trans);
        absTF.transform(p2);
        viewer.addLine(p1,p2, Color.red, radius, "");
        
        // y
        p2 = new Point3d(frame.m01, frame.m11, frame.m21);
        p2.scale(length);
        p2.add(trans);
        absTF.transform(p2);
        viewer.addLine(p1,p2, Color.blue, radius, "");
        
        // z
        p2 = new Point3d(frame.m02, frame.m12, frame.m22);
        p2.scale(length);
        p2.add(trans);
        absTF.transform(p2);
        viewer.addLine(p1,p2, Color.green, radius, "");
    }
    
    private void addBox(RPYXYZ pose, Matrix4d absTF, Size size, Color color, String label){
        Matrix3d frame = RotationUtils.getExtrinsicRotation(pose.rpy());
        Vector3d trans = pose.xyz();
        Point3d location = new Point3d(trans);
        absTF.transform(location);
        //TODO
        // cube ist nicht box, kann also nicht 3 unterschiedlichen Längen haben
        // viewer interface entsprechend erweitern um addBox()?
        Vector3d dir = new Vector3d(absTF.m02, absTF.m12, absTF.m22);
        viewer.addCube(location, dir, size.x(), 
            color, label, false);
    }
    
    private static Point3d[] getPoints(Matrix4d absTF, RPYXYZ pose, double length){
        Vector3d trans = new Vector3d(); 
        absTF.get(trans);
        Point3d absOrigin = new Point3d(pose.xyz());
        absOrigin.add(trans);
        Vector3d dir = new Vector3d();
        Matrix3d rot = new Matrix3d();
        absTF.get(rot);
        rot.getColumn(2, dir);
        dir.normalize();
        dir.scale(length/2d);
        Point3d p1 = new Point3d(absOrigin);
        p1.add(dir);
        Point3d p2 = new Point3d(absOrigin);
        p2.sub(dir);
        return new Point3d[]{p1, p2};
    }
    
    public void addMesh(String path, Matrix4d transform){
        long id = viewer.addMesh(path, transform);
    }
    
    // aus DSL4GA_Impl_Truffle_API App Test Programm
    // ohne annotation API die DSL verwenden und den Visualisierer
    // besser in ein eigenes Projekt verschieben, damit jurdf nicht abbhängig
    // wird von den ganzen DSL... Projekten
    /*private static void invocationTest() throws Exception {
        String source = """
        fn test(a) {
                a, 5
        }

        fn main(a, b) {
                test(b)
                d := getLastListReturn(0)
                e := getLastListReturn(1)
                :p1 := a
                :p2 := b
                a, b, d, e
        }
        """;

        System.out.println("source: " + source);

        try (Program program = new Program(source)) {
                Arguments arguments = new Arguments();
                arguments
                        .sphere_ipns("a", new Point3d(0.2, 0.2, 0.2), 0.2)
                        .sphere_ipns("b", new Point3d(0.5, 0.5, 0.5), 0.2);

                Result answer = program.invoke(arguments);
                double[][] answerScalar = answer.decomposeDoubleArray();

                System.out.println("answer: ");
                for (int i = 0; i < answerScalar.length; ++i) {
                        String current = Arrays.toString(answerScalar[i]);
                        System.out.println(current);
                }
                System.out.println();
        }
    }*/
}
