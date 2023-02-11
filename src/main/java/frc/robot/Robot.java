package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MatrixSetter;
import frc.robot.utils.SmartShuffleboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.matrixSetState().schedule();
  }

  @Override
  public void teleopPeriodic() {
    SmartShuffleboard.put("test", "1", "upLeft", MatrixSetter.isCurrentState(8));
    SmartShuffleboard.put("test", "1", "left", MatrixSetter.isCurrentState(7));
    SmartShuffleboard.put("test", "1", "downLeft", MatrixSetter.isCurrentState(6));

    SmartShuffleboard.put("test", "2", "up", MatrixSetter.isCurrentState(1));
    SmartShuffleboard.put("test", "2", "still", MatrixSetter.isCurrentState(0));
    SmartShuffleboard.put("test", "2", "down", MatrixSetter.isCurrentState(5));

    SmartShuffleboard.put("test", "3", "upRight", MatrixSetter.isCurrentState(2));
    SmartShuffleboard.put("test", "3", "right", MatrixSetter.isCurrentState(3));
    SmartShuffleboard.put("test", "3", "downRight", MatrixSetter.isCurrentState(4));
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
