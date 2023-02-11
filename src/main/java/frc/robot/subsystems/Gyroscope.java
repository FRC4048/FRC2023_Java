package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class Gyroscope extends SubsystemBase {
    AHRS gyroscope = new AHRS();
    private final Field2d robotField = new Field2d();

    public Gyroscope() {
        SmartDashboard.putData(robotField);
    }

    public void periodic() {
        double displacementX = gyroscope.getDisplacementX();
        SmartShuffleboard.put("Odometry", "Pannels", "X Change", displacementX);

        double displacementY = gyroscope.getDisplacementY();
        SmartShuffleboard.put("Odometry", "Pannels", "Y Change", displacementY);

        double gyroAngle = gyroscope.getAngle();
        double convertToRadians = gyroAngle * (Math.PI/180);
        Rotation2d rotation = new Rotation2d(convertToRadians);
        SmartShuffleboard.put("Odometry", "Pannels", "Gyro Angle", gyroAngle);

        //robotField.setRobotPose(displacementX, displacementY, rotation);
        Pose2d pose = new Pose2d(displacementX, displacementY, rotation);
        robotField.setRobotPose(pose);
        
    }
}
