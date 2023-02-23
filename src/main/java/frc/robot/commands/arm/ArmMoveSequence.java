package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmMoveSequence extends SequentialCommandGroup {
    
    private Arm arm;
    private Double angle;

    public ArmMoveSequence(Arm arm, Double angle) {
        arm.setAngle(angle);
        addCommands(
        new VoltageMoveArm(arm, Constants.ARM_RAISE_SPEED, angle),
        new HoldArmPID(arm, angle)
        );
    }
    
}
