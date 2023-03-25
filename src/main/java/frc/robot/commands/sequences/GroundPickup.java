package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;

public class GroundPickup extends SequentialCommandGroup{
    public GroundPickup(Arm arm, Extender extender, GripperSubsystem gripper) {
        setName("-Ground-Pickup");
        addCommands(
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.GROUND_PICKUP.getArmPosition()),
            new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                new ParallelCommandGroup(
                new ExtendToPosition(extender, ArmPositionGrid.GROUND_PICKUP.getExtenderPosition()),
                new OpenGripper(gripper)
                ),
                new HoldArmPID(arm, ArmPositionGrid.GROUND_PICKUP.getArmPosition())
            ), "-Ground-Pickup-drop")
        );

    }
    
}
