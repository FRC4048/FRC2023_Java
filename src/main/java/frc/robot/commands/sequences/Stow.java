package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.Constants;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;

public class Stow extends SequentialCommandGroup {

    public Stow (Arm arm, GripperSubsystem gripper, Extender extender){
        addCommands(
            new ExtendToPosition(extender, 0).withTimeout(3),
            new ParCommandGroupWrapper(new ParallelCommandGroup(
                new CloseGripper(gripper).withTimeout(3),
                new VoltageMoveArm(arm, 0.0, Constants.ARM_STOW_SPEED, 0.0).withTimeout(3)
            ))
            
        );
    }

}
