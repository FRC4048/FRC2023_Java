package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.commands.Autonomous.MoveDistanceOffset;
import frc.robot.commands.Autonomous.MoveDistanceSpinTraj;
import frc.robot.commands.Autonomous.MoveDistanceTraj;
import frc.robot.commands.WaitForSubstationDistance;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.drive.MoveUntilCancledOrTimeout;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;

public class SubstationAutoPickupWithMove extends SequentialCommandGroup {
    public SubstationAutoPickupWithMove(Arm arm, GripperSubsystem gripper, Extender extender, Drivetrain drivetrain) {
        setName("-auto-substation-pickup-");
        addCommands(
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition()),
                new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                    new ParCommandGroupWrapper(new ParallelCommandGroup(
                        new ExtendToPosition(extender, ArmPositionGrid.SUBSTATION_PICKUP.getExtenderPosition()), 
                        new OpenGripper(gripper), 
                        new WaitForSubstationDistance(arm, gripper)
                    ), "-auto-substation-wait-for-dist"),
                    new HoldArmPID(arm, ArmPositionGrid.SUBSTATION_PICKUP.getArmPosition()),
                    new MoveUntilCancledOrTimeout(drivetrain,.3)
                ), "-auto-substation-par-race"),
            new CloseGripper(gripper),
            new VoltageMoveArm(arm, 3, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.SUBSTATION_POST_PICKUP.getArmPosition()),
            new ParallelRaceGroup(
                    new MoveDistanceOffset(drivetrain,-.5,0d,0.5),
                    new HoldArmPID(arm, ArmPositionGrid.SUBSTATION_POST_PICKUP.getArmPosition())
            )
            //slight lift after grab
        );
    }
}
