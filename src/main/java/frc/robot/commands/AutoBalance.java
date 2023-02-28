package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class AutoBalance extends CommandBase{
    Drivetrain drivetrain;
    float lastAng;
    float angle;
    boolean finished;

    public AutoBalance(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        lastAng = 0;
        finished = false;
        addRequirements(drivetrain);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.drive(Constants.AUTO_CHARGESTATION_SPEED, 0, 0, true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        angle = drivetrain.getRoll();
        finished = ((Math.abs(angle - lastAng) > 1) && (Math.abs(angle - lastAng) < 3));
        SmartShuffleboard.put("Driver", "Dif", Math.abs(angle - lastAng));
        SmartShuffleboard.put("Driver", "Finished", finished);
        lastAng = angle;
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
        //drivetrain.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return finished;
        return false;
    }
}
