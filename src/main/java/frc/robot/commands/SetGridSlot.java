package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class SetGridSlot extends CommandBase {
  private final Constants.Grid gridSlot;
  private final RobotContainer container;

  public SetGridSlot(RobotContainer container, Constants.Grid gridSlot) {
    this.gridSlot = gridSlot;
    this.container = container;
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    container.setSelectedGridSlot(gridSlot);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
