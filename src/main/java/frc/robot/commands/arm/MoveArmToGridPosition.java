package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.PieceGrid;
import frc.robot.utils.logging.wrappers.LoggedCommand;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;

public class MoveArmToGridPosition extends LoggedCommand {
     private Arm arm;
     private Extender extender;
     private PieceGrid pieceGrid;

     public MoveArmToGridPosition(Arm arm, Extender extender, PieceGrid pieceGrid) {
          this.arm = arm;
          this.extender = extender;
          this.pieceGrid = pieceGrid;
          setName("-Move-Arm-To-Grid-Position");
     }

     @Override
     public void initialize() {
          super.initialize();
          CommandScheduler.getInstance().schedule(new SequentialCommandGroupWrapper(new ArmMoveSequence(arm, extender, pieceGrid.getSelectedGridSlot().getArmPosition(), pieceGrid.getSelectedGridSlot().getExtenderPosition())));
     }

     @Override
     public void execute() {
     }

     @Override
     public boolean isFinished() {
          return true;
     }
}
