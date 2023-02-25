package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class ChangeSetpoint extends CommandBase{
    
    private Arm arm;
    private DoubleSupplier setpointChange;

    public ChangeSetpoint(Arm arm, DoubleSupplier setpointChange) {
        this.setpointChange = setpointChange;
        this.arm = arm;
    }
    @Override
    public void end(boolean interrupted) {
        
    }
    @Override
    public void execute() {
       arm.setArmSetpoint(-setpointChange.getAsDouble()/4 + arm.getArmSetpoint());
    }
    @Override
    public void initialize() {}
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
