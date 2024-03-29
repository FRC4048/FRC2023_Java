package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.commands.arm.InitialMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;

public class Stow extends SequentialCommandGroup {

    public Stow (Arm arm, GripperSubsystem gripper, Extender extender){
        addCommands(
            new ExtendToPosition(extender, ArmPositionGrid.STOW.getExtenderPosition()),
            new ParCommandGroupWrapper(new ParallelCommandGroup(
                new CloseGripper(gripper),
                new InitialMoveArm(arm, ArmPositionGrid.STOW.getArmPosition())
                ), "-Stow-Lower-Arm")
        );
    }

}
