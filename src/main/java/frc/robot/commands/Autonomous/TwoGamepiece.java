
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
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
        setName("Two Gamepiece Sequence");
        addCommands(
            new ResetEncoders(arm, extender),
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.TOP_RIGHT.getArmPosition()),
            new ParRaceCommandGroupWrapper(
                new ParallelRaceGroup(
                    new SequentialCommandGroupWrapper(
                        new SequentialCommandGroup(
                            new ExtendToPosition(extender, ArmPositionGrid.TOP_RIGHT.getExtenderPosition()),
                            new OpenGripper(gripper)
                        ), "Drop Gamepiece Sequence"),
                    new HoldArmPID(arm, ArmPositionGrid.TOP_RIGHT.getArmPosition())
                )
            ),

            new ParCommandGroupWrapper(
                new ParallelCommandGroup(
                    new SequentialCommandGroupWrapper(
                        new Stow(arm, gripper, extender)
                    ),
                    new MoveDistanceSpinTraj(drivetrain, 0.2, -0.4 * direction, Math.toRadians(180))
                ), "Moving To Pickup"
            ),

            new MoveDistanceSpinTraj(drivetrain, 4.9, .2 * direction , (direction > 0) ? (Math.toRadians(-45)) : (Math.toRadians(45))),
            
            new GroundPickup(arm, extender, gripper),

            new CloseGripper(gripper).withTimeout(1.5),

            new Stow(arm, gripper, extender),

            new MoveDistanceSpinTraj(drivetrain, -4.9, 0, Math.toRadians(180)),

            new MoveDistanceSpinTraj(drivetrain, 0, -.205 * direction, Math.toRadians(180)),
            
            //new MoveDistanceSpinTraj(drivetrain, -0.2, 0.1 * direction, Math.toRadians(180)), Change this
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.TOP_RIGHT.getArmPosition()),
            new ParRaceCommandGroupWrapper(
                new ParallelRaceGroup(
                    new ParCommandGroupWrapper(
                        new ParallelCommandGroup(
                            new ExtendToPosition(extender, ArmPositionGrid.TOP_MIDDLE.getExtenderPosition()),
                            new OpenGripper(gripper)
                        ), "Extend To Drop Off"
                    ),
                    new HoldArmPID(arm, ArmPositionGrid.TOP_MIDDLE.getArmPosition())
                ), "Drop Off On Grid"
            ),
            new Stow(arm, gripper, extender)
        );
    }
}

