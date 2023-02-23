package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagPosition;
import frc.robot.subsystems.Drivetrain;

public class AprilTagSetRotation extends CommandBase {
    private Drivetrain drivetrain;
    private double desiredRotation;
    private AprilTagPosition apriltag;
   
    public AprilTagSetRotation(Drivetrain drivetrain, double desiredRotation, AprilTagPosition apriltag) {
        this.drivetrain = drivetrain;
        this.desiredRotation = desiredRotation;
        this.apriltag = apriltag;

    }

    public void end(boolean interrupted) {
    }

    @Override
    public void initialize() {
        drivetrain.drive(0, 0, desiredRotation-apriltag.getAprilTagRotation(), false);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished() {
        return apriltag.getAprilTagRotation() >= desiredRotation-3 && apriltag.getAprilTagRotation() <= desiredRotation+3;
    }
}