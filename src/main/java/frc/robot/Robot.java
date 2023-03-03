// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.commands.SetGridSlot;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.drive.WheelAlign;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.drive.WheelAlign;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.Diagnostics;
import frc.robot.AutonomousChooser;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static Diagnostics diagnostics;
  private AutonomousChooser locationChooser;
  private int location;
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
    new WheelAlign(m_robotContainer.getDrivetrain()).schedule();
    new ResetGyro(m_robotContainer.getDrivetrain(), 2).schedule();
    new ResetOdometry(m_robotContainer.getDrivetrain(), 0, 13.5, Math.toRadians(180), 3).schedule();
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
  }

  @Override
  public void disabledPeriodic() {
    //add back in after creating these methods
    //autonomousCommand = m_robotContainer.getAutonomousCommand();
    //location = m_robotContainer.getLocation()
    SmartShuffleboard.put("Autonomous", "Chosen Action", m_robotContainer.getAutonomousChooser().getAction().name());
    // if( autonomousCommand != null) {
    //   SmartShuffleboard.put("Autonomous", "Chosen Command", autonomousCommand.getName());
    // }
    // else {
    //   SmartShuffleboard.put("Autonomous", "Chosen Command", " ");
    // }
    // if(location == -1) {
    //   SmartShuffleboard.put("Autonomous", "Chosen Location", "Left");
    // }
    // else if(location == 0) {
    //   SmartShuffleboard.put("Autonomous", "Chosen Location", "Middle");
    // }
    // else if(location == 1) {
    //   SmartShuffleboard.put("Autonomous", "Chosen Location", "Right");
    // }
    // else {
    //   SmartShuffleboard.put("Autonomous", "Chosen Location", " ");
    // }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //add this back in later
    autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    //new WheelAlign(m_robotContainer.getDrivetrain()).schedule();

    m_robotContainer.getArm().zeroPID();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    diagnostics.reset();
    m_robotContainer.getArm().zeroPID();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    diagnostics.refresh();
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION).setKinematics(m_robotContainer.getDrivetrain().getKinematics());

    Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0)),
        new Pose2d(2, 0, new Rotation2d(0)),
        config);
        double time = testTrajectory.getTotalTimeSeconds();
        if (Constants.DRIVETRAIN_DEBUG) {
          SmartShuffleboard.put("BZ", "traj current x", testTrajectory.sample(time).poseMeters.getX());
          SmartShuffleboard.put("BZ", "traj current Vx", testTrajectory.sample(time).velocityMetersPerSecond);
        }
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
