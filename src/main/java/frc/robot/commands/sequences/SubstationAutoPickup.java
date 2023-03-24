package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.WaitForSubstationDistance;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;

public class SubstationAutoPickup extends SequentialCommandGroup {
    public SubstationAutoPickup(Arm arm, GripperSubsystem gripper, Extender extender) {
        setName("-auto-substation-pickup-");
        addCommands(
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, Constants.SUBSTATION_PICKUP_ANGLE),
                new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                    new ParCommandGroupWrapper(new ParallelCommandGroup(
                        new ExtendToPosition(extender, 2000),
                        new OpenGripper(gripper),
                        new WaitForSubstationDistance(arm, gripper)
                    )),
                    new HoldArmPID(arm, Constants.SUBSTATION_PICKUP_ANGLE)
                )),
            new CloseGripper(gripper),
            //slight lift after grab
            new HoldArmPID(arm, Constants.SUBSTATION_PICKUP_ANGLE + 3)
        );
    }
}
