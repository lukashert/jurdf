package de.orat.math.xml.urdf.visual;

/**
 * https://github.com/culurciello/pybullet_ur5_gripper/blob/master/robots/meshes/ur5e/visual/wrist3.dae
 * 
 * A trimesh element specified by a filename, and an optional scale that scales 
 * the mesh's axis-aligned-bounding-box. 
 * 
 * Any geometry format is acceptable but specific application compatibility is 
 * dependent on implementation. The recommended format for best texture and color 
 * support is Collada .dae files. The mesh file is not transferred between 
 * machines referencing the same model. It must be a local file. Prefix the 
 * filename with package://<packagename>/<path> to make the path to the mesh 
 * file relative to the package <packagename>.<p>
 * 
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class Mesh extends Shape {
    
    private String path;
    private double scale;

    public Mesh(String path){
        this.path = path;
    }
    
    public void setScale(double scale){
        this.scale = scale;
    }
    public String getPath(){
        return path;
    }
    @Override
    public String toXML() {
        StringBuilder sb = new StringBuilder();
        sb.append("          <mesh filename=\"");
        sb.append(String.valueOf(path));
        sb.append("\">\n");
        return sb.toString();
    }

    @Override
    public ShapeType getShapeType() {
        return ShapeType.mesh;
    }
}
