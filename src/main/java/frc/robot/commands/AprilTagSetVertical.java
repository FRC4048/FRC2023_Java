package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTagPosition;
import frc.robot.subsystems.Drivetrain;

public class AprilTagSetVertical extends CommandBase{
    private Drivetrain drivetrain;
    private double desiredVertical;
    private AprilTagPosition apriltag;
    private Double currentVertical;
    private double startTime;
   
    public AprilTagSetVertical(Drivetrain drivetrain, double desiredVertical, AprilTagPosition apriltag) {
        this.drivetrain = drivetrain;
        this.desiredVertical = desiredVertical;
        this.apriltag = apriltag;

    }

    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        currentVertical = apriltag.getDistance();
        if (currentVertical == null) {}
        else if (currentVertical > desiredVertical) {
            drivetrain.drive(2.5, 0, 0, false);
        }
        else {
            drivetrain.drive(-2.5, 0, 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        if ((startTime - Timer.getFPGATimestamp()) > 5.00) {
            return true;
        }
       currentVertical = apriltag.getDistance();
       if (currentVertical == null) {
        return true;
       }
       return (Math.abs(currentVertical - desiredVertical) <= Constants.VERTICAL_ERROR_THRESHOLD);
    }
    
}
