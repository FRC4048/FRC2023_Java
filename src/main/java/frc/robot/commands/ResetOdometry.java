package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ResetOdometry extends LoggedCommand {
    private Drivetrain drivetrain;
    private int delay;
    private double startTime;
    private double x, y, rot;

    public ResetOdometry(Drivetrain drivetrain, double x, double y, double rot, int delay){
        this.drivetrain = drivetrain;
        this.delay = delay;
        this.x = x;
        this.y = y;
        this.rot = rot;
        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.resetOdometry(new Pose2d(Units.feetToMeters(x), Units.feetToMeters(y), new Rotation2d(rot)));
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
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
