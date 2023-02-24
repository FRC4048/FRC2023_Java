package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetArmEncoder extends CommandBase {
    private Arm arm;
    private boolean ResetEncoders;
    
    ResetArmEncoder(Arm arm) {
        this.arm = arm;
        
    }
    public void end(boolean interrupted) {
   
    }

    @Override
    public void initialize() {
        arm.resetEncoder();  
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
