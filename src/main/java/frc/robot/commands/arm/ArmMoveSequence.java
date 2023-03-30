package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.utils.logging.wrappers.ParCommandGroupWrapper;

/**
 * DO NOT CALL THIS DIRECTLY MUST BE WRAPPED IN A COMMAND
 */
public class ArmMoveSequence extends SequentialCommandGroup {
    public ArmMoveSequence(Arm arm, Extender extender, double armTargetPosition, double extenderTargetPosition) {
        setName("-Arm-Move");
        if (armTargetPosition > arm.getAnalogValue()) {
            addCommands(
                new InitialMoveArm(arm, armTargetPosition),
                new ParCommandGroupWrapper(new ParallelCommandGroup(
                    new HoldArmPID(arm,armTargetPosition),
                    new ExtendToPosition(extender,extenderTargetPosition)
                ), "-Arm-Move-extend")
            );
        } else {
            addCommands(
                new ExtendToPosition(extender,extenderTargetPosition),
                new InitialMoveArm(arm, armTargetPosition),
                new HoldArmPID(arm,armTargetPosition)
                );
        }
    }

}
