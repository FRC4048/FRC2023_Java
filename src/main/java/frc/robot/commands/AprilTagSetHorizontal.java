package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
    }

    @Override
    public void initialize() {
        drivetrain.drive(0, desiredHorizontal-apriltag.getHorizontalOffset(), 0, false);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished() {
        return apriltag.getHorizontalOffset() >= desiredHorizontal-3 && apriltag.getHorizontalOffset() <= desiredHorizontal+3;
    }
}
