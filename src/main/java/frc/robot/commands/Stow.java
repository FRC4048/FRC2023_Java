package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class Stow extends SequentialCommandGroup {

    public Stow (Arm arm, GripperSubsystem gripper, Extender extender){
        addCommands(
                new ParallelCommandGroup( 
                    new CloseGripper(gripper).withTimeout(2),
                    new ExtendToPosition(extender, 0).withTimeout(2)
            ),
                new SetArmAngle(arm, 0).withTimeout(5)
        );
    }

}
