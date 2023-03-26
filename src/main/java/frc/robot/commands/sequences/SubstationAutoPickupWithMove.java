package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.commands.WaitForSubstationDistance;
import frc.robot.commands.Autonomous.MoveDistanceOffset;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.InitialMoveArm;
import frc.robot.commands.drive.AlignToSubstation;
import frc.robot.commands.drive.MoveUntilCanceledOrTimeout;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;

public class SubstationAutoPickupWithMove extends SequentialCommandGroup {
    public SubstationAutoPickupWithMove(Arm arm, GripperSubsystem gripper, Drivetrain drivetrain) {
        setName("-auto-substation-pickup-with-move-");
        addCommands(
                new AlignToSubstation(drivetrain),
                new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                        new WaitForSubstationDistance(arm, gripper),
                        new HoldArmPID(arm, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition()),
                        new MoveUntilCanceledOrTimeout(drivetrain,.3)
                ), "-auto-substation-move-in"),
                new CloseGripper(gripper),
                new InitialMoveArm(arm, ArmPositionGrid.SUBSTATION_POST_PICKUP.getArmPosition()),
                new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                        new MoveDistanceOffset(drivetrain,-.5,0d,0.5), 
                        new HoldArmPID(arm, ArmPositionGrid.SUBSTATION_POST_PICKUP.getArmPosition())
                ), "-auto-substation-move-backup")
        );
    }
}
