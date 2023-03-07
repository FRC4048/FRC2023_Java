package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetArmEncoder extends CommandBase {
    private Arm arm;
    
    public ResetArmEncoder(Arm arm) {
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
