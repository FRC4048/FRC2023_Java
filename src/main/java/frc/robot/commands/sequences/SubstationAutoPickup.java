package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoCloseGripper;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class SubstationAutoPickup extends SequentialCommandGroup {
    public SubstationAutoPickup(Arm arm, GripperSubsystem gripper, Extender extender) {
        addCommands(
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, 31.5),
                new ParallelRaceGroup(
                    new ParallelCommandGroup(
                        new ExtendToPosition(extender, 2000),
                        new OpenGripper(gripper),
                        new AutoCloseGripper(arm, gripper)
                    ),
                    new HoldArmPID(arm, 31.5)
                ),
            new HoldArmPID(arm, 35.0)
        );
    }
}
