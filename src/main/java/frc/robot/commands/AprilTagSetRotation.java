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

        drivetrain.drive(0, 0, 0, false);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        double currentRotation = apriltag.getRotation();
        if (desiredRotation > currentRotation) {
            // Desired is greater than current, turn left
            drivetrain.drive(0, 0, .2, false);
        } else {
            // Desired is less than current, turn right
            drivetrain.drive(0, 0, -.2, false);
        }
    }

    @Override
    public boolean isFinished() {
        double currentRotation = apriltag.getRotation();
        if (Math.abs(desiredRotation - currentRotation) < .017) { 
            return true;
        }
        return false;
    }
}