package frc.robot.apriltags;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagMap {
  
    public static List<AprilTag> getAprilPosList(Alliance alliance) {
        if (alliance == Alliance.Red) {

            List<AprilTag> apriltags = new ArrayList<>();
            apriltags.add(0, new AprilTag(1, new Pose3d(1.02743, 6.938264, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 0)))));
            apriltags.add(1, new AprilTag(2, new Pose3d(1.02743, 5.261864, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 0)))));
            apriltags.add(2, new AprilTag(3, new Pose3d(1.02743, 3.585464, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 0)))));
            apriltags.add(3, new AprilTag(4, new Pose3d(0.36195, 1.260094, 0.695452, new Rotation3d(new Quaternion(0, 0, 0, 0)))));
            apriltags.add(4, new AprilTag(5, new Pose3d(16.178784, 1.260094, 0.695452, new Rotation3d(new Quaternion(0, 0, 0, 1.0))))); 
            apriltags.add(5, new AprilTag(6, new Pose3d(15.513558, 3.585464, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1.0)))));
            apriltags.add(6, new AprilTag(7, new Pose3d(15.513558, 5.261864, 0.462788,  new Rotation3d(new Quaternion(0, 0, 0, 1.0)))));
            apriltags.add(7, new AprilTag(8, new Pose3d(15.513558, 6.938264, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1.0)))));

            // AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(apriltags, 16.54175, 8.0137);
            // return aprilTagFieldLayout;
            return apriltags;
        } else {
            List<AprilTag> apriltags = new ArrayList<>();
            apriltags.add(0, new AprilTag(1, new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1.0)))));
            apriltags.add(1, new AprilTag(2, new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1.0)))));
            apriltags.add(2, new AprilTag(3, new Pose3d(15.513558, 4.424426, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 1.0)))));
            apriltags.add(3, new AprilTag(4, new Pose3d(16.178784, 6.749796, 0.695452, new Rotation3d(new Quaternion(0, 0, 0, 1.0)))));
            apriltags.add(4, new AprilTag(5, new Pose3d(0.36195, 6.749796, 0.695452, new Rotation3d(new Quaternion(0, 0, 0, 0))))); 
            apriltags.add(5, new AprilTag(6, new Pose3d(1.02743, 4.424426, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 0)))));
            apriltags.add(6, new AprilTag(7, new Pose3d(1.02743, 2.748026, 0.462788,  new Rotation3d(new Quaternion(0, 0, 0, 0)))));
            apriltags.add(7, new AprilTag(8, new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(new Quaternion(0, 0, 0, 0)))));

            return apriltags;
            // AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(apriltags, 16.54175, 8.0137);
            // return aprilTagFieldLayout;
        }

    }
    //Getter to return aprilTagList

    public static AprilTagFieldLayout getAprilTagLayout(List<AprilTag> aprilTags){
        AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, 16.54175, 8.0137);
        return aprilTagFieldLayout;
    }

}
