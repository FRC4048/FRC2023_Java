// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MatrixSetter;
import frc.robot.commands.SetArmAngle;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer robotContainer;
  private Arm arm;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    arm = robotContainer.getArm();
    //SmartShuffleboard.putCommand("Diag", "Reset", new WheelAlign(robotContainer.getDrivetrain()));
    //SmartShuffleboard.putCommand("Drive", "Move", new Forward(robotContainer.getDrivetrain()));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartShuffleboard.putCommand("PID", "setAngle=0", new SetArmAngle(arm, 0));
    SmartShuffleboard.putCommand("PID", "setAngle=90", new SetArmAngle(arm, 90));
    SmartShuffleboard.putCommand("PID", "setAngle=180", new SetArmAngle(arm, 180));

    SmartShuffleboard.put("PID", "encoder", Math.toDegrees(Math.toDegrees(arm.getEncoderValue())));

    SmartShuffleboard.put("PID", "angle", arm.getAngle());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //SmartShuffleboard.put("Diag", "Abs Encoder", "FR", robotContainer.getDrivetrain().m_frontRight.absEncoder.getPosition());

    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
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
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
