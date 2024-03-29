package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.InitialMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;

public class GroundPickup extends SequentialCommandGroup{
    public GroundPickup(Arm arm, Extender extender, GripperSubsystem gripper) {
        setName("-Ground-Pickup");
        addCommands(
            new InitialMoveArm(arm, ArmPositionGrid.GROUND_PICKUP.getArmPosition()),
            new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                new ParCommandGroupWrapper(new ParallelCommandGroup(
                    new ExtendToPosition(extender, ArmPositionGrid.GROUND_PICKUP.getExtenderPosition()),
                    new OpenGripper(gripper)
                ),"-Ground-Pickup-position"),
                new HoldArmPID(arm, ArmPositionGrid.GROUND_PICKUP.getArmPosition())
            ), "-Ground-Pickup-position-hold")
        );
    }
}
