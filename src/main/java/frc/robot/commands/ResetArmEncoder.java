package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetArmEncoder extends CommandBase {
    private Arm arm;
    private boolean ResetEncoders;
    
    ResetArmEncoder(Arm arm, boolean ResetEncoders) {
        this.arm = arm;
        this.ResetEncoders = ResetEncoders;
        
    }
    public void end(boolean interrupted) {
   
    }

    @Override
    public void initialize() {
        if (ResetEncoders == true) {
            arm.resetEncoder();
        }     
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
