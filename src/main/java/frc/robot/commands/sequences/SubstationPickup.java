package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;

public class SubstationPickup extends SequentialCommandGroup {
    public SubstationPickup(Arm arm, GripperSubsystem gripper) {
        setName("SubstationPickup"); //Not used?
        addCommands(
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, 32.0),
                new ParCommandGroupWrapper(new ParallelCommandGroup(
                    new OpenGripper(gripper),
                    new HoldArmPID(arm, 32.0)
                ))
        );
    }
}
