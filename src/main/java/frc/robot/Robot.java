// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.drive.Forward;
import frc.robot.commands.drive.WheelAlign;
import frc.robot.commands.SetArmAngle;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.Diagnostics;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static Diagnostics diagnostics;
  private Arm arm;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    diagnostics = new Diagnostics();
    m_robotContainer = new RobotContainer();
    SmartShuffleboard.putCommand("Drive", "Move", new Forward(m_robotContainer.getDrivetrain()));
    SmartShuffleboard.putCommand("Drive", "ResetGyro", new ResetGyro(m_robotContainer.getDrivetrain(), 0));
    new WheelAlign(m_robotContainer.getDrivetrain()).schedule();
    new ResetGyro(m_robotContainer.getDrivetrain(), 2).schedule();
    arm = m_robotContainer.getArm();
    //SmartShuffleboard.putCommand("Diag", "Reset", new WheelAlign(m_robotContainer.getDrivetrain()));
    //SmartShuffleboard.putCommand("Drive", "Move", new Forward(m_robotContainer.getDrivetrain()));
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
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //SmartShuffleboard.put("Diag", "Abs Encoder", "FR", m_robotContainer.getDrivetrain().m_frontRight.absEncoder.getPosition());


    //add this back in later
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //new WheelAlign(m_robotContainer.getDrivetrain()).schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION).setKinematics(m_robotContainer.getDrivetrain().getKinematics());

    Trajectory testTrajectory = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(new Translation2d(1, 0)),
        new Pose2d(2, 0, new Rotation2d(0)),
        config);
        double time = testTrajectory.getTotalTimeSeconds();
        SmartShuffleboard.put("BZ", "traj current x", testTrajectory.sample(time).poseMeters.getX());
        SmartShuffleboard.put("BZ", "traj current Vx", testTrajectory.sample(time).velocityMetersPerSecond);
  }

  public static Diagnostics getDiagnostics() {
    return diagnostics;
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
