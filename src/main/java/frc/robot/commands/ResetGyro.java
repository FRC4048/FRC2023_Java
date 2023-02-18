package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.SmartShuffleboard;

public class ResetGyro extends CommandBase {
    private Drivetrain drivetrain;
    private int delay;
    private double startTime;

    public ResetGyro(Drivetrain drivetrain, int delay){
        this.drivetrain = drivetrain;
        this.delay = delay;
        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.resetGyro();
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
