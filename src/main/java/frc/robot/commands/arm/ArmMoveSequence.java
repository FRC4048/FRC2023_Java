package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.PieceGrid;

/**
 * The button binding should not call this directly
 */
public class ArmMoveSequence extends SequentialCommandGroup {
    public ArmMoveSequence(Arm arm, Extender extender, double armTargetPosition, double extenderTargetPosition) {
        if (armTargetPosition > arm.getEncoderValue()) {
            addCommands(
                new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, armTargetPosition), 
                new ParallelCommandGroup(
                    new HoldArmPID(arm,armTargetPosition),
                    new ExtendToPosition(extender,extenderTargetPosition)
                )
            );
        }
        else {
            addCommands(
                new ExtendToPosition(extender,extenderTargetPosition),
                new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, armTargetPosition), 
                new HoldArmPID(arm,armTargetPosition)
            );
        }
    }

}
