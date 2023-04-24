package frc.robot.commands;

import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.subsystems.PieceGrid;
import frc.robot.utils.Logger;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class SetGridSlot extends LoggedCommand {
  private final ArmPositionGrid gridSlot;
  private final PieceGrid pieceGrid;

  public SetGridSlot(PieceGrid pieceGrid, ArmPositionGrid gridSlot) {
    this.gridSlot = gridSlot;
    this.pieceGrid = pieceGrid;
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    pieceGrid.setSelectedGridSlot(gridSlot);
    Logger.logString("/pieceGrid/selection", gridSlot.name(), Constants.ENABLE_LOGGING);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
