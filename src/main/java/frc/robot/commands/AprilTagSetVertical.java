package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagPosition;
import frc.robot.subsystems.Drivetrain;

public class AprilTagSetVertical extends CommandBase{
    private Drivetrain drivetrain;
    private double desiredVertical;
    private AprilTagPosition apriltag;
   
    public AprilTagSetVertical(Drivetrain drivetrain, double desiredVertical, AprilTagPosition apriltag) {
        this.drivetrain = drivetrain;
        this.desiredVertical = desiredVertical;
        this.apriltag = apriltag;

    }

    public void end(boolean interrupted) {
    }

    @Override
    public void initialize() {
        drivetrain.drive(desiredVertical-apriltag.getAprilTagVertical(), 0, 0, false);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished() {
        return apriltag.getAprilTagVertical() >= desiredVertical-3 && apriltag.getAprilTagVertical() <= desiredVertical+3;
    }
    
}
