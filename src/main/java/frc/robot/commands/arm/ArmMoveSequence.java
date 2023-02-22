package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class ArmMoveSequence extends SequentialCommandGroup {
    
    private Arm arm;
    private Double angle;

    public ArmMoveSequence(Arm arm, Double angle) {
        addCommands(
        new VoltageMoveArm(arm, 3.0, angle),
        new HoldArmPID(arm, angle)
        );
    }
    
}
