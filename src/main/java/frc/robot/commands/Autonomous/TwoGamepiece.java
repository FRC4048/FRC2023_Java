
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        setName("-Auto-2GP-");
        addCommands(
            new SequentialCommandGroupWrapper(new ResetEncoders(arm, extender),"-Auto-2GP-Reset-Encoders"),            
            new InitialMoveArm(arm, ArmPositionGrid.TOP_RIGHT.getArmPosition()),
            new ParRaceCommandGroupWrapper(
                new ParallelRaceGroup(
                    new SequentialCommandGroupWrapper(
                        new SequentialCommandGroup(
                            new ExtendToPosition(extender, ArmPositionGrid.TOP_RIGHT.getExtenderPosition()),
                            new OpenGripper(gripper)
                        ), "-Auto-2GP-drop-seq-"),
                    new HoldArmPID(arm, ArmPositionGrid.TOP_RIGHT.getArmPosition())
                ), "-Auto-2GP-deposit"
            ),

            new ParCommandGroupWrapper(
                new ParallelCommandGroup(
                    new SequentialCommandGroupWrapper(new Stow(arm, gripper, extender), "-Auto-2GP-stow"),
                    new MoveDistanceSpinTraj(drivetrain, 0.2, -0.4 * direction, Math.toRadians(180))
                ), "-Auto-2GP-diag-move-"
            ),

            new MoveDistanceSpinTraj(drivetrain, 4.9, .4 * direction , (direction > 0) ? (Math.toRadians(-45)) : (Math.toRadians(45))),
            
            new GroundPickup(arm, extender, gripper),

            new WaitCommand(.5),

            new CloseGripper(gripper),

            new SequentialCommandGroupWrapper(new Stow(arm, gripper, extender), "-Auto-2gp-stow"),

            new MoveDistanceSpinTraj(drivetrain, -4.9, -.4 * direction, Math.toRadians(180))
        );
    }
}

