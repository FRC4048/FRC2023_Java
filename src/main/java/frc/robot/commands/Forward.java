package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


public class Forward extends CommandBase{
    private Drivetrain drivetrain;
    public double InitEncValue;
    public Forward(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setPower(Constants.DRIVE_BACK_LEFT_D, 0);
        drivetrain.setPower(Constants.DRIVE_BACK_RIGHT_D, 0);
        //drivetrain.setPower(Constants.DRIVE_FRONT_LEFT_D, 0);
        drivetrain.setPower(Constants.DRIVE_FRONT_RIGHT_D, 0);
    }

    @Override
    public void initialize() {
        InitEncValue=drivetrain.getRelEnc(Constants.DRIVE_BACK_LEFT_D);

    }

    @Override
    public void execute(){
        drivetrain.setPower(Constants.DRIVE_BACK_LEFT_D, 0.1);
        drivetrain.setPower(Constants.DRIVE_BACK_RIGHT_D, 0.1);
        drivetrain.setPower(Constants.DRIVE_FRONT_LEFT_D, 0.1);
        drivetrain.setPower(Constants.DRIVE_FRONT_RIGHT_D, 0.1);
    }

    @Override
    public boolean isFinished() {   
        return drivetrain.getRelEnc(Constants.DRIVE_BACK_LEFT_D)-InitEncValue>Constants.WHEEL_RADIUS*2*Math.PI;
    }   
}