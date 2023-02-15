package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PIDDrive;
import frc.robot.utils.SmartShuffleboard;

public class ArmController extends CommandBase{
    private double desiredAngle;
    private PIDDrive pidDrive;

    public ArmController(Drivetrain drivetrain, double change) {
        pidDrive = new PIDDrive();
        desiredAngle = pidDrive.getAngle();
    }


    @Override
    public void execute() {
        pidDrive.setAngle(desiredAngle);
        SmartShuffleboard.put("pid", "angle", desiredAngle);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
