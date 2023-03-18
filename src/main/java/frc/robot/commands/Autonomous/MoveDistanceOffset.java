package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveDistanceOffset extends CommandBase{
    private Drivetrain drivetrain;
    private double changeX, changeY, startPosX, startPosY, speedX, speedY, maxSpeed, startTime, ratioCalc;
    

    public MoveDistanceOffset(Drivetrain drivetrain, double changeX, double changeY, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.changeX = changeX;
        this.changeY = changeY;
        this.maxSpeed = maxSpeed;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        startPosX = drivetrain.getPoseX();
        startPosY = drivetrain.getPoseY();

        if(Math.abs(changeX) > Math.abs(changeY)) {
            speedX = Math.signum(changeX) * maxSpeed;
            ratioCalc = changeX/maxSpeed;
            speedY = changeY/ratioCalc;
        }
        else {
            speedY = Math.signum(changeY) * maxSpeed;
            ratioCalc = changeY/maxSpeed;
            speedX = changeX/ratioCalc;
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
        if(Math.abs(startPosX - drivetrain.getPoseX()) > Math.abs(changeX) && Math.abs(startPosY - drivetrain.getPoseY()) > Math.abs(changeY)) {
            return true;
        }
        return (Timer.getFPGATimestamp() - startTime) > Constants.MOVE_OFFSET_TIMEOUT;
    }
}
