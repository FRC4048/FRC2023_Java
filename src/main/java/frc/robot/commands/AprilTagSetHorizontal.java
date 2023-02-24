package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTagPosition;
import frc.robot.subsystems.Drivetrain;

public class AprilTagSetHorizontal extends CommandBase {

    private Drivetrain drivetrain;
    private double desiredHorizontal;
    private AprilTagPosition apriltag;
    double startTime;

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
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        Double currentHorizontalOffset = apriltag.getHorizontalOffset();
        if (currentHorizontalOffset != null) {
            if (desiredHorizontal > currentHorizontalOffset) {
                // desired more than current, must move right
                drivetrain.drive(0, -0.2, 0, false);
            } else {
                // desired less than current, move left
                drivetrain.drive(0, 0.2, 0, false);
            }
        } else {
            drivetrain.drive(0, 0, 0, false);
        }

    }

    @Override
    public boolean isFinished() {
        if ((Timer.getFPGATimestamp() - startTime) > 5.00) {
            return true;
        }
        Double currentHorizontalOffset = apriltag.getHorizontalOffset();
        double currentTime = System.currentTimeMillis();
        if ((currentTime - startTime) > 5000) {
            return true;
        }

        if (currentHorizontalOffset != null) {
            if (Math.abs(desiredHorizontal - currentHorizontalOffset) < Constants.HORIZONTAL_ERROR_THRESHOLD) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
}
