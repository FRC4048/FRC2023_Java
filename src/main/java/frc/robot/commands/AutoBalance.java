package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

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
    private int counterStage1;
    private int counterStage2;
    private int counterStage3;
    private boolean firstMax;
    private boolean firstMin;
    private boolean secondMax;
    private List<Float> frame;
    boolean finished;

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
        finished = false;
        minAngle = 42;         //🐭
        maxAngle = 0;
        finishedCounter = 0;
        minCounter = 0;
        frame = new LinkedList<Float>();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      SmartShuffleboard.put("Driver", "FirstMax", firstMax);
      SmartShuffleboard.put("Driver", "FirstMin", firstMin);
      SmartShuffleboard.put("Driver", "SecondMax", secondMax);
      SmartShuffleboard.put("Driver", "Finished", finished);
      SmartShuffleboard.put("Driver", "Stage 1", counterStage1);
      SmartShuffleboard.put("Driver", "Stage 2", counterStage2);
      SmartShuffleboard.put("Driver", "Stage 3", counterStage3);

      updateFrame();

        if (firstMax) {
            firstMin = Math.abs(drivetrain.getFilterRoll()) > 13;
            firstMax = !firstMin;
            drivetrain.drive(.7, 0, 0, true);
            counterStage1++;
        }

        if (firstMin) {
            /*if (Math.abs(drivetrain.getFilterRoll()) < minAngle) {
              minAngle = Math.abs(drivetrain.getFilterRoll());
            } else if (Math.abs(drivetrain.getFilterRoll()) - minAngle >= 1) {
              minCounter++;
            }*/
            clearFrame();
            //climbCheck();
            flatCheck();
            firstMin = !secondMax;

            drivetrain.drive(.5, 0, 0, true);
            counterStage2++;
        }

        if (secondMax) {
            clearFrame();
            drivetrain.drive(Constants.AUTO_CHARGESTATION_SPEED, 0, 0, true);
            if (Math.abs(drivetrain.getFilterRoll()) > maxAngle) {
                maxAngle = Math.abs(drivetrain.getFilterRoll());
            }

            finished = tipCheck();
            counterStage3++;
            /*if (maxAngle - Math.abs(drivetrain.getFilterRoll()) > 1) {
                finishedCounter++;
            } else {
                finishedCounter = 0;
            }*/
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
        return finished;
    }

    private void updateFrame() {
      frame.add(Math.abs(drivetrain.getFilterRoll()));
      if (frame.size() > 10) {
        frame.remove(0);
      }
    }

    private void clearFrame() {
      frame.clear();
    }

    private void climbCheck() {
        secondMax = (frame.size() >= 10) && ((frame.get(0) < frame.get(3)) && (frame.get(3) < frame.get(7)) && (frame.get(7) < frame.get(9))); 
    }

    private void flatCheck() {
      secondMax = (frame.size() >= 10) && (frame.get(9) - frame.get(0) < 1) && (frame.get(0) > 9);
    }

    private boolean tipCheck() {
      return (frame.size() >= 10) && ((frame.get(0) > frame.get(3)) && (frame.get(3) > frame.get(7)) && (frame.get(7) > frame.get(9)));
    }
}
