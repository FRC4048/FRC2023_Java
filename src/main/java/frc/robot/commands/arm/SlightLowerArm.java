package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.PieceGrid;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class SlightLowerArm extends LoggedCommand{
     private Arm arm;
     private Extender extender;
     private PieceGrid pieceGrid;

     public SlightLowerArm(Arm arm, Extender extender, PieceGrid pieceGrid) {
          this.arm = arm;
          this.extender = extender;
          this.pieceGrid = pieceGrid;
     }

     @Override
     public void initialize() {
          super.initialize();
          CommandScheduler.getInstance().schedule(
          new HoldArmPID(arm, pieceGrid.getSelectedGridSlot().getArmPosition() - Constants.ARM_LOWER_DISTANCE));     
          //new ArmMoveSequence(arm, extender, pieceGrid.getSelectedGridSlot().getArmPosition() - Constants.ARM_LOWER_DISTANCE, pieceGrid.getSelectedGridSlot().getExtenderPosition()));
     }

     @Override
     public boolean isFinished() {
          return true;
     }
}
