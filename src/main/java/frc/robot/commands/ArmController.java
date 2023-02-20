package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmController extends CommandBase{
    private double desiredAngle;
    private Arm arm;
    private double change;

    public ArmController(Arm arm, double change) {
        this.change = change;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        desiredAngle = arm.getAngle() + change;
        
        if(desiredAngle > Constants.ARM_MAX_ANGLE) {
            desiredAngle = Constants.ARM_MAX_ANGLE;
        }
        
        arm.setAngle(desiredAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}