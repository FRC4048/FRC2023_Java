package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.PieceGrid;
import frc.robot.subsystems.ProtectionMechanism;
import frc.robot.utils.SmartShuffleboard;

/**
 * DO NOT CALL THIS DIRECTLY MUST BE WRAPPED IN A COMMAND
 */
public class ArmMoveSequence extends ParallelCommandGroup {
    public ArmMoveSequence(Arm arm, Extender extender, double armTargetPosition, double extenderTargetPosition) {
        addCommands(
            new SequentialCommandGroup(
                new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE, armTargetPosition), 
                new HoldArmPID(arm,armTargetPosition)
            ),new ExtendToPosition(extender,extenderTargetPosition),
                new CommandBase(){
            @Override
            public void execute() {
                SmartShuffleboard.put("Arm","estAngle", ProtectionMechanism.angleFromEncoder(arm.getEncoderValue()));
                SmartShuffleboard.put("Arm","extMax", ProtectionMechanism.maxEnc(arm.getEncoderValue()));
            }
            @Override
            public boolean isFinished() {
                return  true;
            }
        }
        );
    }

}
