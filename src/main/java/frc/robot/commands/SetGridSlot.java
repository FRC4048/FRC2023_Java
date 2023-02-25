package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmPositionGrid;
import frc.robot.subsystems.PieceGrid;

public class SetGridSlot extends CommandBase {
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
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
