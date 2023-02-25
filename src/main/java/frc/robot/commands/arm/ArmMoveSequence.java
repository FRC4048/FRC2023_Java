package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.PieceGrid;

public class ArmMoveSequence extends SequentialCommandGroup {
    public ArmMoveSequence(Arm arm, Extender extender, PieceGrid pieceGrid) {
        addCommands(
        new VoltageMoveArm(arm, 1d, pieceGrid.getSelectedGridSlot().getArmPosition()),
        new ParallelCommandGroup(
                new HoldArmPID(arm,pieceGrid.getSelectedGridSlot().getArmPosition()),
                new ExtendToPosition(extender,pieceGrid.getSelectedGridSlot().getExtenderPosition()))
        );
    }

}
