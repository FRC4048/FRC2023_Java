package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;


public class Move extends CommandBase{
    private Drivetrain drivetrain;
    private double startEnc;
    private double distance;

    public Move(Drivetrain drivetrain, double distance){
        this.drivetrain = drivetrain;
        this.distance = distance;
        addRequirements(drivetrain);
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setPower(Constants.DRIVE_BACK_LEFT_D, 0);
        drivetrain.setPower(Constants.DRIVE_BACK_RIGHT_D, 0);
        drivetrain.setPower(Constants.DRIVE_FRONT_LEFT_D, 0);
        drivetrain.setPower(Constants.DRIVE_FRONT_RIGHT_D, 0);
    }

    @Override
    public void initialize() {
        startEnc = drivetrain.getRelEnc(Constants.DRIVE_BACK_LEFT_D);

    }

    @Override
    public void execute(){
        drivetrain.setPower(Constants.DRIVE_BACK_LEFT_D, 0.2 * Math.signum(distance));
        drivetrain.setPower(Constants.DRIVE_BACK_RIGHT_D, 0.2 * Math.signum(distance));
        drivetrain.setPower(Constants.DRIVE_FRONT_LEFT_D, 0.2 * Math.signum(distance));
        drivetrain.setPower(Constants.DRIVE_FRONT_RIGHT_D, 0.2 * Math.signum(distance));
    }

    @Override
    public boolean isFinished() {   
        return drivetrain.getRelEnc(Constants.DRIVE_BACK_LEFT_D) - startEnc >= distance/(Constants.WHEEL_RADIUS * 2 * Math.PI);
    }   
}