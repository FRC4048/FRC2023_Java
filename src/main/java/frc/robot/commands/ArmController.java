package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDDrive;

public class ArmController extends CommandBase{
    private double desiredAngle;
    private PIDDrive pidDrive;
    private double change;

    public ArmController(PIDDrive pidDrive, double change) {
        this.change = change;
        this.pidDrive = pidDrive;
        addRequirements(pidDrive);
    }

    @Override
    public void execute() {
        desiredAngle = pidDrive.getAngle() + change;
        
        if(desiredAngle > 180) {
            desiredAngle = 180;
        }
        if(desiredAngle < 0) {
            desiredAngle = 0;
        }
        
        pidDrive.setAngle(desiredAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}