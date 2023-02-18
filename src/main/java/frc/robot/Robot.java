// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.Forward;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.drive.WheelAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Command m_wheelAlign;

  private boolean wheelsAligned;

  public double frontLeftOffset;
  public double frontRightOffset;
  public double backLeftOffset;
  public double backRightOffset;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_wheelAlign = m_robotContainer.getWheelAlign();
    wheelsAligned=false;
    SmartShuffleboard.putCommand("Diag", "Reset", new WheelAlign(m_robotContainer.getDrivetrain()));
    SmartShuffleboard.putCommand("Drive", "Move", new Forward(m_robotContainer.getDrivetrain()));
    SmartShuffleboard.putCommand("Drive", "ResetGyro", new ResetGyro(m_robotContainer.getDrivetrain()));
    m_robotContainer.getDrivetrain().resetGyro();

    frontLeftOffset=m_robotContainer.getDrivetrain().getM_frontLeftTurn().getEncoder().getPosition()- Math.toRadians(m_robotContainer.getDrivetrain().getFrontLeftCanCoder().getAbsolutePosition());
    frontRightOffset=m_robotContainer.getDrivetrain().getM_frontRightTurn().getEncoder().getPosition()- Math.toRadians(m_robotContainer.getDrivetrain().getFrontRightCanCoder().getAbsolutePosition());
    backLeftOffset=m_robotContainer.getDrivetrain().getM_backLeftTurn().getEncoder().getPosition()- Math.toRadians(m_robotContainer.getDrivetrain().getBackLeftCanCoder().getAbsolutePosition());
    backRightOffset=m_robotContainer.getDrivetrain().getM_backRightTurn().getEncoder().getPosition()- Math.toRadians(m_robotContainer.getDrivetrain().getBackRightCanCoder().getAbsolutePosition());
    SmartShuffleboard.put("Diag", "FL Offset", frontLeftOffset);
    SmartShuffleboard.put("Diag", "FR Offset", frontRightOffset);
    SmartShuffleboard.put("Diag", "BL Offset", backLeftOffset);
    SmartShuffleboard.put("Diag", "BR Offset", backRightOffset);
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    SmartShuffleboard.putCommand("Drive", "ResetGyro", new ResetGyro(m_robotContainer.getDrivetrain()));
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //SmartShuffleboard.put("Diag", "Abs Encoder", "FR", m_robotContainer.getDrivetrain().m_frontRight.absEncoder.getPosition());

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

//    if (wheelsAligned == false){
//      m_wheelAlign.schedule();
//      wheelsAligned = true;
//    }
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

//    if (wheelsAligned == false){
//      m_wheelAlign.schedule();
//    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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
