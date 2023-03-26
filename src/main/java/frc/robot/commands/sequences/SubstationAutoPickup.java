package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.commands.WaitForSubstationDistance;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.InitialMoveArm;
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
            new InitialMoveArm(arm, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition()),
                new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                    new ParCommandGroupWrapper(new ParallelCommandGroup(
                        new ExtendToPosition(extender, ArmPositionGrid.SUBSTATION_PICKUP.getExtenderPosition()), 
                        new OpenGripper(gripper), 
                        new WaitForSubstationDistance(arm, gripper)
                    ), "-auto-substation-wait-for-dist"),
                    new HoldArmPID(arm, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition())
                ), "-auto-substation-par-race"),
            new CloseGripper(gripper),
            //slight lift after grab
            new HoldArmPID(arm, ArmPositionGrid.SUBSTATION_POST_PICKUP.getArmPosition())
        );
    }
}
