package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.GripperPos;

public class GripperPosition extends CommandBase {
    private final GripperPos position;
    private final GripperSubsystem gripper;
    private double speed = 0;
    public GripperPosition(GripperSubsystem gripper, GripperPos position) { 
        this.position = position;
        this.gripper = gripper;
    }
    @Override

    public void end(boolean interrupted) {
    
    }

    @Override
    public void initialize() {
        if (gripper.gripperPosition() >= position.pos) {
            
        }
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished() {   
        return false;
    }   
}
