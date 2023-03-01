package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class AutoBalance extends CommandBase{
    Drivetrain drivetrain;
    float maxAngle;
    float minAngle;
    int finishedCounter;
    boolean firstMax;
    boolean firstMin;
    boolean secondMax;

    public AutoBalance(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        firstMax = true;
        firstMin = false;
        secondMax = false;
        minAngle = 20;
        maxAngle = 0;
        finishedCounter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (firstMax) {
            firstMin = Math.abs(drivetrain.getFilterRoll()) > 15;
            firstMax = !firstMin;
            drivetrain.drive(.3, 0, 0, true);
        }

        if (firstMin) {
            secondMax = Math.abs(drivetrain.getFilterRoll()) < 8;
            firstMin = !secondMax;
            drivetrain.drive(.05, 0, 0, true);
        }

        if (secondMax) {
            drivetrain.drive(Constants.AUTO_CHARGESTATION_SPEED, 0, 0, true);
            if (Math.abs(drivetrain.getFilterRoll()) > maxAngle) {
                maxAngle = Math.abs(drivetrain.getFilterRoll());
            }

            if (maxAngle - Math.abs(drivetrain.getFilterRoll()) > 2) {
                finishedCounter++;
            } else {
                finishedCounter = 0;
            }
        }
        //DO NOT DELETE THIS BANGER BY MATVEY

        /*while(!((accelX < 0.1 && accelX > -0.1) || (accelY < 0.1 && accelY > -0.1)))// while the robot is NOT balanced (EXACT ACCELERATION BOUNDARIES UNKNOWN)
        {   
            speedX = accelToSpeed(accelX);
            speedY = accelToSpeed(accelY);
            drivetrain.drive(speedX, speedY, 0, true);   
        }
        drivetrain.drive(0, 0, 0, true);*/

        //DO NOT DELETE THIS BANGER BY MATVEY
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        new DriveForTime(drivetrain, -.2, .1).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finishedCounter > 5;
    }
}
