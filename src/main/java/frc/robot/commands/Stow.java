package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class Stow extends SequentialCommandGroup {

    public Stow (Arm arm, GripperSubsystem gripper, Extender extender){
        addCommands(
                new CloseGripper(gripper),
                new ExtendToPosition(extender, 0),
                new VoltageMoveArm(arm, Constants.ARM_STOW_SPEED, 0.0)
        );
    }

}
