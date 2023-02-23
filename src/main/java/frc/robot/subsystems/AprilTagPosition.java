package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagPosition extends SubsystemBase{



    private double AprilTagRotation = 10;
    private double AprilTagHorizontal = 10;
    private double AprilTagVertical = 10;
    
    public double getAprilTagRotation() {
        return AprilTagRotation;
    }
    
    public double getAprilTagHorizontal() {
        return AprilTagHorizontal;
    }
    
    public double getAprilTagVertical() {
        return AprilTagVertical;
    }
    
}
