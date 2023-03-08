package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class AutoBalance extends CommandBase{
    private Drivetrain drivetrain;
    private float maxAngle;
    private float minAngle;
    private int finishedCounter;
    private int minCounter;
    private boolean firstMax;
    private boolean firstMin;
    private boolean secondMax;
    private double startTime;

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
        minAngle = 42;
        maxAngle = 0;
        finishedCounter = 0;
        minCounter = 0;
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      SmartShuffleboard.put("Driver", "FirstMax", firstMax);
      SmartShuffleboard.put("Driver", "FirstMin", firstMin);
      SmartShuffleboard.put("Driver", "SecondMax", secondMax);
        if (firstMax) {
            firstMin = Math.abs(drivetrain.getFilterRoll()) > 15;
            firstMax = !firstMin;
            drivetrain.drive(Constants.BALANCE_STEEP_SPEED, 0, 0, true);
        }

        if (firstMin) {
            //secondMax = Math.abs(drivetrain.getFilterRoll()) < 8;
            //firstMin = !secondMax;
            if (Math.abs(drivetrain.getFilterRoll()) < minAngle) {
              minAngle = Math.abs(drivetrain.getFilterRoll());
            } else if (Math.abs(drivetrain.getFilterRoll()) - minAngle >= 1) {
              minCounter++;
            }
            
            secondMax = minCounter > 10;
            firstMin = !secondMax;

            drivetrain.drive(Constants.BALANCE_LOW_SPEED, 0, 0, true);
        }

        if (secondMax) {
            drivetrain.drive(Constants.AUTO_CHARGESTATION_SPEED, 0, 0, true);
            if (Math.abs(drivetrain.getFilterRoll()) > maxAngle) {
                maxAngle = Math.abs(drivetrain.getFilterRoll());
            }

            if (maxAngle - Math.abs(drivetrain.getFilterRoll()) > 1) {
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
        new DriveForTime(drivetrain, -.4, .1).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (finishedCounter > Constants.CHARGESTATION_BALANCED) || ((Timer.getFPGATimestamp() - startTime) > Constants.CHARGESTATION_TIMEOUT);
    }
}
