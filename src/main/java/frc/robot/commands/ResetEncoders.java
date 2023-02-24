package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.extender.ResetExtenderEncoder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class ResetEncoders extends SequentialCommandGroup{
    public ResetEncoders (Arm arm, GripperSubsystem gripper, Extender extender){
        addCommands(
                new ParallelCommandGroup( 
                    new ResetExtenderEncoder(extender).withTimeout(3),
                    new ResetArm(arm).withTimeout(3)
            ),
                
                new ResetArmEncoder(arm)
                
        );
        
    }
}
