package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.PieceGrid;

public class MoveArmToGridPosition extends CommandBase {
     private Arm arm;
     private Extender extender;
     private PieceGrid pieceGrid;

     public MoveArmToGridPosition(Arm arm, Extender extender, PieceGrid pieceGrid) {
          this.arm = arm;
          this.extender = extender;
          this.pieceGrid = pieceGrid;
     }

     @Override
     public void initialize() {
          CommandScheduler.getInstance().schedule(new ArmMoveSequence(arm, extender, pieceGrid.getSelectedGridSlot().getArmPosition(), pieceGrid.getSelectedGridSlot().getExtenderPosition()));
     }

     @Override
     public void execute() {
     }

     @Override
     public boolean isFinished() {
          return true;
     }
}
