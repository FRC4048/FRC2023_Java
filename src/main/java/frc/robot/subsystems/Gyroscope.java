package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class Gyroscope extends SubsystemBase {
    AHRS gyroscope = new AHRS();
    public Gyroscope() {}

    public void periodic() {
        double displacementX = gyroscope.getDisplacementX();
        SmartShuffleboard.put("Odometry", "Pannels", "X Change", displacementX);

        double displacementY = gyroscope.getDisplacementY();
        SmartShuffleboard.put("Odometry", "Pannels", "Y Change", displacementY);

        double gyroAngle = gyroscope.getAngle();
        SmartShuffleboard.put("Odometry", "Pannels", "Gyro Angle", gyroAngle);
    }
}
