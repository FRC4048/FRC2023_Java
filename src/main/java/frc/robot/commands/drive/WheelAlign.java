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
    private double startTime;

    private SwerveModuleState frontLeftSwerveModuleState;
    private SwerveModuleState frontRightSwerveModuleState;
    private SwerveModuleState backLeftSwerveModuleState;
    private SwerveModuleState backRightSwerveModuleState;

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
        frontLeftSwerveModuleState= new SwerveModuleState(
            0, 
            new Rotation2d(calculateAngle(drivetrain.getM_frontLeftTurn().getEncoder().getPosition(), Math.toRadians(Constants.FRONT_LEFT_ABS_ENCODER_ZERO), Math.toRadians(drivetrain.getFrontLeftCanCoder().getAbsolutePosition()))));
        
        frontRightSwerveModuleState= new SwerveModuleState(
            0, 
            new Rotation2d(calculateAngle(drivetrain.getM_frontRightTurn().getEncoder().getPosition(), Math.toRadians(Constants.FRONT_RIGHT_ABS_ENCODER_ZERO), Math.toRadians(drivetrain.getFrontRightCanCoder().getAbsolutePosition()))));
        
        backLeftSwerveModuleState= new SwerveModuleState(
            0, 
            new Rotation2d(calculateAngle(drivetrain.getM_backLeftTurn().getEncoder().getPosition(), Math.toRadians(Constants.BACK_LEFT_ABS_ENCODER_ZERO), Math.toRadians(drivetrain.getBackLeftCanCoder().getAbsolutePosition()))));
        
        backRightSwerveModuleState= new SwerveModuleState(
            0, 
            new Rotation2d(calculateAngle(drivetrain.getM_backRightTurn().getEncoder().getPosition(), Math.toRadians(Constants.BACK_RIGHT_ABS_ENCODER_ZERO), Math.toRadians(drivetrain.getBackRightCanCoder().getAbsolutePosition()))));        
        


        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        drivetrain.getM_frontLeft().setDesiredState(frontLeftSwerveModuleState);
        drivetrain.getM_frontRight().setDesiredState(frontRightSwerveModuleState);
        drivetrain.getM_backLeft().setDesiredState(backLeftSwerveModuleState);
        drivetrain.getM_backRight().setDesiredState(backRightSwerveModuleState);
        //SmartShuffleboard.put("Test", "Desired Angle", backRightSwerveModuleState.angle);
    }
    

    @Override
    public boolean isFinished() {
        return MathUtil.applyDeadband(Math.toRadians(drivetrain.getFrontLeftCanCoder().getAbsolutePosition()) - Math.toRadians(Constants.FRONT_LEFT_ABS_ENCODER_ZERO), 0.1) == 0 && MathUtil.applyDeadband(Math.toRadians(drivetrain.getFrontRightCanCoder().getAbsolutePosition()) - Math.toRadians(Constants.FRONT_RIGHT_ABS_ENCODER_ZERO), 0.1) == 0 && MathUtil.applyDeadband(Math.toRadians(drivetrain.getBackLeftCanCoder().getAbsolutePosition()) - Math.toRadians(Constants.BACK_LEFT_ABS_ENCODER_ZERO), 0.1) == 0 && MathUtil.applyDeadband(Math.toRadians(drivetrain.getBackRightCanCoder().getAbsolutePosition()) - Math.toRadians(Constants.BACK_RIGHT_ABS_ENCODER_ZERO), 0.1) == 0 || Timer.getFPGATimestamp() - startTime > 3;

    }  
   
    public double calculateAngle(double RelEncoderPos, double DesiredAbsEncPos, double StartingAbsEncPos) {
        return ((DesiredAbsEncPos-StartingAbsEncPos)+RelEncoderPos);
    } 
}
