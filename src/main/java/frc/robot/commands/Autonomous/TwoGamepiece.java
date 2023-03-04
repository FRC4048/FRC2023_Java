
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.commands.sequences.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;


public class TwoGamepiece extends SequentialCommandGroup {
    public double rotation;
    public TwoGamepiece(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper, double rotation) {
        this.rotation = rotation;
        addCommands(
            new ResetOdometry(drivetrain, 0, 0, rotation, 0),
            new ResetEncoders(arm, extender),
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.TOP_RIGHT.getArmPosition()),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ExtendToPosition(extender, ArmPositionGrid.TOP_RIGHT.getExtenderPosition()),
                    new OpenGripper(gripper)
                ),
                new HoldArmPID(arm, ArmPositionGrid.TOP_RIGHT.getArmPosition())
            ),
            new ParallelCommandGroup(
                new Stow(arm, gripper, extender),
                new MoveDistanceSpinTraj(drivetrain, 0.2, -0.2, 0)
            ),
            new MoveDistanceSpinTraj(drivetrain, 2, 0, Math.toRadians(180)),
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, 9.0),
            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new ExtendToPosition(extender, 2000.0),
                    new OpenGripper(gripper)
                    ),
                new HoldArmPID(arm, 9.0)
            ),
            new ParallelCommandGroup( 
                new Stow(arm, gripper, extender),
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    new MoveDistanceSpinTraj(drivetrain, -2, 0, Math.toRadians(180))
                )
            ),
            new MoveDistanceSpinTraj(drivetrain, -0.2, 0.2, 0),
            new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.TOP_RIGHT.getArmPosition()),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ExtendToPosition(extender, ArmPositionGrid.TOP_RIGHT.getExtenderPosition()),
                    new OpenGripper(gripper)
                ),
                new HoldArmPID(arm, ArmPositionGrid.TOP_RIGHT.getArmPosition())
            ),
            new Stow(arm, gripper, extender)
        );
    }
}

