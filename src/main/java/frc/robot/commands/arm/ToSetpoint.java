package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ToSetpoint extends CommandBase{
    private double setpoint;
    private Arm arm;
    public ToSetpoint(Arm arm, double setpoint) {
        this.setpoint = setpoint;
        this.arm = arm;
    }
    @Override
    public void end(boolean interrupted) {
        
    }
    @Override
    public void execute() {
        
    }
    @Override
    public void initialize() {
        arm.setArmSetpoint(setpoint);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    
    
}
