package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTagPosition;
import frc.robot.subsystems.Drivetrain;

public class AprilTagSetHorizontal extends CommandBase {

    private Drivetrain drivetrain;
    private double desiredHorizontal;
    private AprilTagPosition apriltag;

    public AprilTagSetHorizontal(Drivetrain drivetrain, double desiredHorizontal, AprilTagPosition apriltag) {
        this.drivetrain = drivetrain;
        this.desiredHorizontal = desiredHorizontal;
        this.apriltag = apriltag;

    }

    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double currentHorizontalOffset = apriltag.getHorizontalOffset();
        if (desiredHorizontal > currentHorizontalOffset) {
            // desired more than current, must move right
            drivetrain.drive(0, -0.2, 0, false);
        } else {
            // desired less than current, move left
            drivetrain.drive(0, 0.2, 0, false);
        }

    }

    @Override
    public boolean isFinished() {
        double currentHorizontalOffset = apriltag.getHorizontalOffset();
        if (Math.abs(desiredHorizontal - currentHorizontalOffset) < Constants.HORIZONTAL_ERROR_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }
}
