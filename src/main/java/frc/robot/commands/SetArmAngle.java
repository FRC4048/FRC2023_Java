package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmAngle extends CommandBase {
    private double degrees;
    private Arm arm;


    public SetArmAngle(Arm arm, double degrees) {
        this.arm = arm;
        this.degrees = degrees;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setAngle(degrees);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
