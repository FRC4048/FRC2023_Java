package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GyroOffseter extends CommandBase {
    private Drivetrain drivetrain;
    private double offset;
    public GyroOffseter(Drivetrain drivetrain, double offset) {
        this.drivetrain = drivetrain;
        this.offset = offset;
    }

    public void end(boolean interrupted) {

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        drivetrain.setGyroOffset(drivetrain.getGyroOffset()+ offset);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
