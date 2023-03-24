package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.commands.LookForSubstation;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class SubstationPickup extends SequentialCommandGroup {
    public SubstationPickup(Arm arm, GripperSubsystem gripper, Extender extender) {
        addCommands(
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition()),
            new HoldArmPID(arm, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition()),
                new SequentialCommandGroup(
                    new ExtendToPosition(extender, ArmPositionGrid.SUBSTATION_PICKUP.getExtenderPosition()),
                    new OpenGripper(gripper),
                    new LookForSubstation()         
                ),
                new SequentialCommandGroup(
                    new CloseGripper(gripper),
                    new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition())
                )
        );
    }
}
