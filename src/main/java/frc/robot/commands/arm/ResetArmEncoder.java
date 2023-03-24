package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ResetArmEncoder extends LoggedCommand {
    private Arm arm;
    
    public ResetArmEncoder(Arm arm) {
        this.arm = arm;
        
    }

    @Override
    public void initialize() {
        super.initialize();
        arm.resetEncoder();  
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
