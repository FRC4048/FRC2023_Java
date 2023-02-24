package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetGridSlot extends CommandBase {
  private Constants.Grid gridSlot;
  private Arm arm;

  public SetGridSlot(Arm arm, Constants.Grid gridSlot) { 
    this.gridSlot = gridSlot;
    this.arm = arm;
  }


  @Override
  public boolean isFinished() {
    return true;
  }
}
