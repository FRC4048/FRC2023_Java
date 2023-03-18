package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmPositionGrid;
import frc.robot.subsystems.PieceGrid;

public class LookForSubstation extends CommandBase {
    private int timeout = 5;
    private double startTime;
  public LookForSubstation() {

  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime > 5);
  }
}
