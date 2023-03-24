package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class LookForSubstation extends LoggedCommand {
    private int timeout = 5;
    private double startTime;
  public LookForSubstation() {

  }

  @Override
  public void initialize() {
    super.initialize();
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime > 5);
  }
}
