package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class GyroOffseter extends LoggedCommand {
    private Drivetrain drivetrain;
    private double offset;
    public GyroOffseter(Drivetrain drivetrain, double offset) {
        this.drivetrain = drivetrain;
        this.offset = offset;
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
