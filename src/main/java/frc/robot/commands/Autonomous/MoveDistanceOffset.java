package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveDistanceOffset extends CommandBase{
    private Drivetrain drivetrain;
    private double changeX, changeY, startPosX, startPosY, speedX, speedY, maxSpeed;

    public MoveDistanceOffset(Drivetrain drivetrain, double changeX, double changeY, double speedX, double speedY, double speed, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.changeX = changeX;
        this.changeY = changeY;
        this.speedX = speedX;
        this.speedY = speedY;
        this.maxSpeed = maxSpeed;
    }

    @Override
    public void initialize() {
        startPosX = drivetrain.getPoseX();
        startPosY = drivetrain.getPoseY();

        if(changeX > changeY) {
            speedX = maxSpeed;
            speedY = speedX * (changeY/changeX);
        }
        else {
            speedY = maxSpeed;
            speedX = speedY * (changeX/changeY);
        }
        
    }

    @Override
    public void execute() {
        drivetrain.drive(speedX, speedY, 0.0, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(startPosX - drivetrain.getPoseX()) > Constants.SUBSTATION_DRIVE_BACK_DISTANCE && Math.abs(startPosY - drivetrain.getPoseY()) > Constants.SUBSTATION_DRIVE_BACK_DISTANCE) {
            return true;
        }
        return false;
    }
}
