package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, Constants.SUBSTATION_PICKUP_ANGLE),
            new HoldArmPID(arm, Constants.SUBSTATION_PICKUP_ANGLE),
                new SequentialCommandGroup(
                    new ExtendToPosition(extender, Constants.SUBSTATION_PICKUP_EXTENSION),
                    new OpenGripper(gripper),
                    new LookForSubstation()         
                ),
                new SequentialCommandGroup(
                    new CloseGripper(gripper),
                    new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, Constants.SUBSTATION_POST_PICKUP_ANGLE)
                )
        );
    }
}
