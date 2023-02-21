package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmAngle extends CommandBase {
    private double degrees;
    private Arm arm;

    //this is used to set the desired angle of the arm. The pid will then seek this value.
    //range is approx. 0-1100, but you probably shouldn't send it higher than 1000.
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
