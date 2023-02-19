package frc.robot.commands.drive;

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

public class WheelAlign extends CommandBase {
    private Drivetrain drivetrain;

    public WheelAlign(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        drivetrain.getM_frontLeft().setSteerOffset(Constants.FRONT_LEFT_ABS_ENCODER_ZERO);
        drivetrain.getM_frontRight().setSteerOffset(Constants.FRONT_RIGHT_ABS_ENCODER_ZERO);
        drivetrain.getM_backLeft().setSteerOffset(Constants.BACK_LEFT_ABS_ENCODER_ZERO);
        drivetrain.getM_backRight().setSteerOffset(Constants.BACK_RIGHT_ABS_ENCODER_ZERO);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
