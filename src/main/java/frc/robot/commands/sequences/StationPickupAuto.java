package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SubstationTrajAllign;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.luxonis.LuxonisVision;

public class StationPickupAuto extends SequentialCommandGroup {
    public StationPickupAuto(Arm arm, GripperSubsystem gripper, Drivetrain drivetrain, LuxonisVision luxonisVision, Extender extender) {
        addCommands(
            new SubstationTrajAllign(drivetrain, luxonisVision, 1.0),
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, 32.0),
            new ParallelRaceGroup(
                new HoldArmPID(arm, 32.0),
                new SequentialCommandGroup(
                    new OpenGripper(gripper),
                    new ExtendToPosition(extender, 3000.0), //change this number later
                    new CloseGripper(gripper),
                    new ExtendToPosition(extender, 0)
                )
            ),
            new Stow(arm, gripper, extender)
        );
    }
}

 