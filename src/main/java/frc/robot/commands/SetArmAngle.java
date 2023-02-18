package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmMotor;

public class SetArmAngle extends CommandBase {
    private double degrees;
    private ArmMotor armMotor;


    public SetArmAngle(ArmMotor armMotor, double degrees) {
        this.armMotor = armMotor;
        this.degrees = degrees;
        addRequirements(armMotor);
    }

    @Override
    public void initialize() {
        armMotor.setAngle(degrees);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
