package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class GroundPickup extends SequentialCommandGroup{
    public GroundPickup(Arm arm, Extender extender, GripperSubsystem gripper) {
        addCommands(
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, 9.0),
                new ParallelCommandGroup(
                    new ExtendToPosition(extender, 2000.0),
                    new OpenGripper(gripper),
                    new HoldArmPID(arm, 20.0)
                )
        );

    }
    
}