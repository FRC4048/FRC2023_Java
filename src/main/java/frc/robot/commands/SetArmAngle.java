package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDDrive;

public class SetArmAngle extends CommandBase {
    private double degrees;
    private PIDDrive pidDrive;


    public SetArmAngle(PIDDrive pidDrive, double degrees) {
        this.pidDrive = pidDrive;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        pidDrive.setAngle(degrees);

    }

    @Override
    public void execute() {
        
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
