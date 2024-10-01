package de.orat.math.xml.urdf.util;

import de.orat.math.xml.urdf.api.Chain.DH;
import de.orat.math.xml.urdf.util.ParameterizedLine.IntersectionParameters;
import java.util.ArrayList;
import java.util.List;
import org.jogamp.vecmath.AxisAngle4d;
import org.jogamp.vecmath.Matrix3d;
import org.jogamp.vecmath.Matrix4d;
import org.jogamp.vecmath.Point3d;
import org.jogamp.vecmath.Vector3d;

/**
 * @author Oliver Rettig (Oliver.Rettig@orat.de)
 */
public class DHUtils2 {
 
    // wegen correctAxis() sollte das nur für ur korrekt sein
    // mit FK hab ichs getestet, sollte daher grundsätzlich funktionieren
    // aber warum stimmen das mit dem urdf-Dateien für ur5e nicht überein?
    //TODO
    /**
     * Creates a list/sequence of Tranformation corresponding to the given
     * list of Denavit-Hartenberg segments.
     * 
     * @param segments list of Denavit-Hartenberg segments/parameter sets
     * @return 
     */
    public static List<Matrix4d> to(List<DH> segments, boolean withFakeLinks){
        List<Matrix4d> chain = buildChain(segments);
        if (!withFakeLinks){
            //correctAxis(chain, segments, 1);
            //correctAxis(chain, segments, 2);
            return getSimplified(chain);
        } else {
            return chain;
        }
    }
    

    /**
     * Correct axes for segments with d==0.
     * 
     * Each DH-Segment is split into two chain segments. One representing the d 
     * and theta parameters and one with the a and alpha parameters. If we start 
     * from the first segment (which represents d and theta), there follows a 
     * passive segment (with a and alpha) and the next d/theta-segment after that.
     *
     * In principle, the d parameter of the first segment gets set to zero, first. 
     * With this change, the kinematic structure gets destroyed, which has to be 
     * corrected:
     * 
     * - With setting d to 0, both the start and end points of the passive 
     *   segment move along the rotational axis of the start segment. Instead, 
     *   the end point of the passive segment has to move along the rotational 
     *   axis of the next segment. This creates a change in a and and theta, if
     *   the two rotational axes are not parallel.
     *
     * - The length of moving along the next segment's rotational axis is 
     *   calculated by intersecting the rotational axis with the XY-plane of the 
     *   first segment.
     * 
     * TODO
     * - dann könnte ich die Methode doch vorsichtshalber für alle indizes
     *   aufrufen und nicht nur für 1,2 oder?
     */
    private static void correctAxis(List<Matrix4d> chain, List<DH> segments, int linkIndex){
 
        //if (segments[2 * link_index](2, 3) == 0.0){
        if (chain.get(2 * linkIndex).m23 == 0.0){ // z-translation==d==0, z-axes are parallel
          // Nothing to do here.
          return;
        }

        //Eigen::Matrix4d fk_current = Eigen::Matrix4d::Identity();
        Matrix4d fk_current = new Matrix4d();
        fk_current.setIdentity();
        
        //Eigen::Vector3d current_passive = Eigen::Vector3d::Zero();
        Point3d current_passive = new Point3d();
        
        //Eigen::Matrix4d fk_next_passive = Eigen::Matrix4d::Identity();
        Matrix4d fk_next_passive = new Matrix4d();
        fk_next_passive.setIdentity();
        
        //fk_next_passive *= chain_[link_index * 2] * chain_[link_index * 2 + 1];
        fk_next_passive.mul(chain.get(linkIndex*2));
        fk_next_passive.mul(chain.get(linkIndex*2+1));
        
        //Eigen::Vector3d eigen_passive = fk_next_passive.topRightCorner(3, 1);
        Vector3d eigen_passive = new Vector3d();
        fk_next_passive.get(eigen_passive);
        
        //Eigen::Vector3d eigen_next = (fk_next_passive * chain_[(link_index + 1) * 2]).topRightCorner(3, 1);
        Vector3d eigen_next = new Vector3d();
        Matrix4d temp = new Matrix4d(fk_next_passive);
        temp.mul(chain.get((linkIndex + 1) * 2));
        temp.get(eigen_next);
        
        // Construct a representation of the next segment's rotational axis
        //Eigen::ParametrizedLine<double, 3> next_line;
        //next_line = Eigen::ParametrizedLine<double, 3>::Through(eigen_passive, eigen_next);
        ParameterizedLine next_line = new ParameterizedLine(new Point3d(eigen_passive), 
                                                            new Point3d(eigen_next));
       
        // XY-Plane of first segment's start
        //Eigen::Hyperplane<double, 3> plane(fk_current.topLeftCorner(3, 3) * Eigen::Vector3d(0, 0, 1), 
        // current_passive);
        Vector3d rot_z = new Vector3d(0,0,1);
        fk_current.transform(rot_z);
        Plane4d plane = new Plane4d(rot_z, current_passive);
        
        // Intersect the rotation axis with the XY-Plane. We use both notations, the length and
        // intersection point, as we will need both. The intersection_param is the length moving along the
        // plane (change in the next segment's d param), while the intersection point will be used for
        // calculating the new angle theta.
        //double intersection_param = next_line.intersectionParameter(plane);
        //Eigen::Vector3d intersection = next_line.intersectionPoint(plane) - current_passive;
        //Point3d intersection = next_line.intersectionPoint(plane);
        IntersectionParameters intersection_param = next_line.intersectionParameters(plane);
        Point3d intersection = intersection_param.p();
        //double new_theta = std::atan(intersection.y() / intersection.x());
        double new_theta = Math.atan(intersection.y/intersection.x);
        
        // Upper and lower arm segments on URs all have negative length due to dh params
        double new_length = -1 * (new Vector3d(intersection).length());
        
        
        //ROS_DEBUG_STREAM("Wrist line intersecting at " << std::endl << intersection);
        //ROS_DEBUG_STREAM("Angle is " << new_theta);
        //ROS_DEBUG_STREAM("Length is " << new_length);
        //ROS_DEBUG_STREAM("Intersection param is " << intersection_param);

        // as we move the passive segment towards the first segment, we have to move away the next segment
        // again, to keep the same kinematic structure.
        double sign_dir = next_line.direction().z > 0 ? 1.0 : -1.0;
        double distance_correction = intersection_param.t() * sign_dir;

        // Set d parameter of the first segment to 0 and theta to the calculated new angle
        // Correct arm segment length and angle
        //chain_[2 * link_index](2, 3) = 0.0;
        chain.get(2 * linkIndex).setElement(2, 3, 0d);
        
        //chain_[2 * link_index].topLeftCorner(3, 3) =
        //    Eigen::AngleAxisd(new_theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        chain.get(2 * linkIndex).setRotation(
            new AxisAngle4d(new Vector3d(0d,0d,1d), new_theta));

        // Correct arm segment length and angle
        //chain_[2 * link_index + 1](0, 3) = new_length;
        chain.get(2 * linkIndex + 1).setElement(0, 3, new_length);
        
        //chain_[2 * link_index + 1].topLeftCorner(3, 3) =
        //    Eigen::AngleAxisd(robot_parameters_.segments_[link_index].theta_ - new_theta, Eigen::Vector3d::UnitZ())
        //        .toRotationMatrix() *
        //    Eigen::AngleAxisd(robot_parameters_.segments_[link_index].alpha_, Eigen::Vector3d::UnitX()).toRotationMatrix();

        Matrix3d m1 = new Matrix3d();
        m1.set(new AxisAngle4d(new Vector3d(0d,0d,1d), segments.get(linkIndex).theta() - new_theta));
        Matrix3d m2 = new Matrix3d();
        m2.set(new AxisAngle4d(new Vector3d(1d,0d,0d), segments.get(linkIndex).alpha()));
        m1.mul(m2);
        chain.get(2 * linkIndex + 1).setRotation(m1);
        
        // Correct next joint
        Matrix4d mat = chain.get(2 * linkIndex + 2);
        mat.setElement(2, 3, mat.m23 - distance_correction);
    }

  
    /**
     * Get the transformation matrix representaton of the chain as constructed
     * by the dh-parameters.
     * 
     * This will contain twice as many transformation matrices as joints, as for
     * each set of dh-parameters two matrices are generated. If you´d like to
     * receive one matrix per joint instead, use the getSimplified() method.<p>
     * 
     * @param segments
     * @return 
     */
    private static List<Matrix4d> buildChain(List<DH> segments){
      List<Matrix4d> chain = new ArrayList<>();
      for (int i=0;i<segments.size();++i){
        DH dh = segments.get(i);
        Matrix4d seg1 = new Matrix4d();
        seg1.setIdentity();
        seg1.setRotation(new AxisAngle4d(0d,0d,1d, dh.theta()));
        seg1.setTranslation(new Vector3d(0,0,dh.d()));
        chain.add(seg1);

        Matrix4d seg2 = new Matrix4d();
        seg2.setIdentity();
        seg2.setRotation(new AxisAngle4d(1d,0d,0d, dh.alpha()));
        seg2.setTranslation(new Vector3d(dh.r(),0d,0d));
        chain.add(seg2);
      }
      return chain;
    }

    /**
     * Get the first matrix, and substitute further matrices by multiplying two
     * matrices by its product.
     * 
     * @param chain
     * @return simplified chain
     */
    private static List<Matrix4d> getSimplified(List<Matrix4d> chain){
      List<Matrix4d> result = new ArrayList<>();
      // add first active (theta, d) trafo
      result.add(new Matrix4d(chain.get(0)));

      for (int i=1;i<chain.size()-1; i+=2){
        // add passive trafo (alpha, r) * next active trafo (theta, d)
        Matrix4d m = new Matrix4d(chain.get(i));
        m.mul(chain.get(i+1));
        result.add(m);
      }

      // add last passive trafo (alpha, r)
      result.add(chain.get(chain.size()-1));
      return result;
    }
    
    public static Matrix4d calcForwardKinematics(List<Matrix4d> simplified_chain, 
                                              double[] joint_values, int link_nr) {
        
        // Currently ignore input and calculate for zero vector input
        //Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
        Matrix4d result = new Matrix4d();
        result.setIdentity();

        //std::vector<Eigen::Matrix4d> simplified_chain = getSimplified();
        //for (size_t i = 0; i < link_nr; ++i)
        for (int i=0;i<link_nr;i++) {
          //Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
          Matrix4d rotation = new Matrix4d();
          rotation.setIdentity();
          
          //rotation.topLeftCorner(3, 3) = Eigen::AngleAxisd(joint_values(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
          rotation.set(new AxisAngle4d(new Vector3d(0d,0d,1d), joint_values[i]));
          
          //output *= simplified_chain[i] * rotation;
          result.mul(simplified_chain.get(i));
          result.mul(rotation);
        }

        return result;
    }
}
