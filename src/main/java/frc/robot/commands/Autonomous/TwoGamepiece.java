
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.InitialMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.commands.sequences.GroundPickup;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.commands.sequences.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;


public class TwoGamepiece extends SequentialCommandGroup {
    public double rotation;

    public TwoGamepiece(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper, double direction) {
        setName("-auto-two-gamepiece-");
        addCommands(
            new ResetEncoders(arm, extender),
            new InitialMoveArm(arm, ArmPositionGrid.TOP_RIGHT.getArmPosition()),
            new ParRaceCommandGroupWrapper(
                new ParallelRaceGroup(
                    new SequentialCommandGroupWrapper(
                        new SequentialCommandGroup(
                            new ExtendToPosition(extender, ArmPositionGrid.TOP_RIGHT.getExtenderPosition()),
                            new OpenGripper(gripper)
                        ), "-auto-two-drop-seq-"),
                    new HoldArmPID(arm, ArmPositionGrid.TOP_RIGHT.getArmPosition())
                )
            ),

            new ParCommandGroupWrapper(
                new ParallelCommandGroup(
                    new SequentialCommandGroupWrapper(
                        new Stow(arm, gripper, extender)
                    ),
                    new MoveDistanceSpinTraj(drivetrain, 0.2, -0.4 * direction, Math.toRadians(180))
                ), "-auto-two-diag-move-"
            ),

            new MoveDistanceSpinTraj(drivetrain, 5.1, .3 * direction , (direction > 0) ? (Math.toRadians(-45)) : (Math.toRadians(45))),
            
            new GroundPickup(arm, extender, gripper),

            new CloseGripper(gripper),

            new Stow(arm, gripper, extender),

            new MoveDistanceSpinTraj(drivetrain, -4.9, -.4 * direction, Math.toRadians(180))
        );
    }
}

