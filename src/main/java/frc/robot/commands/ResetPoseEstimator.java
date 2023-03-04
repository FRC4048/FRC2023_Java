package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetPoseEstimator extends CommandBase {
    private Drivetrain drivetrain;
    private int delay;
    private double startTime;
    private double x, y, rot;

    public ResetPoseEstimator(Drivetrain drivetrain, double x, double y, double rot, int delay){
        this.drivetrain = drivetrain;
        this.delay = delay;
        this.x = x;
        this.y = y;
        this.rot = rot;
        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.resetPoseEstimator(new Pose2d(Units.feetToMeters(x), Units.feetToMeters(y), new Rotation2d(rot)));
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
    }
    

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) >= delay;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
