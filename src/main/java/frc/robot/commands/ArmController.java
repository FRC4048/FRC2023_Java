package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmMotor;

public class ArmController extends CommandBase{
    private double desiredAngle;
    private ArmMotor armMotor;
    private double change;

    public ArmController(ArmMotor armMotor, double change) {
        this.change = change;
        this.armMotor = armMotor;
        addRequirements(armMotor);
    }

    @Override
    public void execute() {
        desiredAngle = armMotor.getAngle() + change;
        
        if(desiredAngle > 180) {
            desiredAngle = 180;
        }
        if(desiredAngle < 0) {
            desiredAngle = 0;
        }
        
        armMotor.setAngle(desiredAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}