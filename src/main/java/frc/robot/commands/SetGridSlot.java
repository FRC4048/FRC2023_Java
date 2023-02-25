package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Grid;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PieceGrid;
import frc.robot.utils.SmartShuffleboard;

public class SetGridSlot extends CommandBase {
  private final Grid gridSlot;
  private final PieceGrid pieceGrid;

  public SetGridSlot(PieceGrid pieceGrid, Grid gridSlot) {
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
