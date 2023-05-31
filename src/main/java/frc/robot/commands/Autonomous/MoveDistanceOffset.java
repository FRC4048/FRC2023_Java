package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Odometry;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class MoveDistanceOffset extends LoggedCommand{
    private Drivetrain drivetrain;
    private Odometry odometry;
    private double changeX, changeY, startPosX, startPosY, speedX, speedY, maxSpeed, startTime, ratioCalc;
    

    public MoveDistanceOffset(Drivetrain drivetrain, Odometry odometry, double changeX, double changeY, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.changeX = changeX;
        this.changeY = changeY;
        this.maxSpeed = maxSpeed;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        
        startTime = Timer.getFPGATimestamp();

        startPosX = odometry.getPoseX();
        startPosY = odometry.getPoseY();

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
        super.end(interrupted);
        drivetrain.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(startPosX - odometry.getPoseX()) > Math.abs(changeX) && Math.abs(startPosY - odometry.getPoseY()) > Math.abs(changeY)) {
            return true;
        }
        return (Timer.getFPGATimestamp() - startTime) > Constants.MOVE_OFFSET_TIMEOUT;
    }
}
