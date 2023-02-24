package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase{
    Drivetrain drivetrain;
    double accelX;
    double accelY;
    double speedX;
    double speedY;

    public AutoBalance(Drivetrain drivetrain, double accelX, double accelY){
        this.drivetrain = drivetrain;
        this.accelX = accelX;
        this.accelY = accelY;
    }

    public double accelToSpeed(double accel) //converting acceleration value we get into speed (EXACT VALUES UNKOWN)
    {
        return accel * 0.1;
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        while(!((accelX < 0.1 && accelX > -0.1) || (accelY < 0.1 && accelY > -0.1)))// while the robot is NOT balanced (EXACT ACCELERATION BOUNDARIES UNKNOWN)
        {   
            speedX = accelToSpeed(accelX);
            speedY = accelToSpeed(accelY);
            drivetrain.drive(speedX, speedY, 0, true);   
        }
        drivetrain.drive(0, 0, 0, true);   
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (accelX == 0 && accelY == 0);
    }
}
