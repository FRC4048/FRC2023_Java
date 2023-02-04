package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class WheelAlign extends CommandBase {
    private Drivetrain drivetrain;
    double startTime;

    public WheelAlign(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPower(Constants.DRIVE_BACK_LEFT_S, 0);
        drivetrain.setPower(Constants.DRIVE_BACK_RIGHT_S, 0);
        drivetrain.setPower(Constants.DRIVE_FRONT_LEFT_S, 0);
        drivetrain.setPower(Constants.DRIVE_FRONT_RIGHT_S, 0);
        drivetrain.SetRelEncZero();
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        drivetrain.AlignWheel();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 5;
    }   
}
