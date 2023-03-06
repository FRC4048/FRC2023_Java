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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.commands.SetGridSlot;
import frc.robot.commands.ChangeLedID;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.drive.WheelAlign;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.drive.WheelAlign;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LedPanel;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.Diagnostics;
import frc.robot.AutonomousChooser;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer m_robotContainer;
  private static Diagnostics diagnostics;
 
  @Override
  public void robotInit() {
    diagnostics = new Diagnostics();
    m_robotContainer = new RobotContainer();
    new WheelAlign(m_robotContainer.getDrivetrain()).schedule();
    new ResetGyro(m_robotContainer.getDrivetrain(), 2).schedule();
    new ResetOdometry(m_robotContainer.getDrivetrain(), 0, 13.5, Math.toRadians(180), 3).schedule();
      }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    SmartShuffleboard.put("Autonomous", "Chosen Action, Location", m_robotContainer.getAutonomousChooser().getAction().name() + ", " + m_robotContainer.getAutonomousChooser().getLocation().name());

  }

  @Override
  public void autonomousInit() {
    autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }    
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    m_robotContainer.getArm().zeroPID();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    diagnostics.reset();
    m_robotContainer.getArm().zeroPID();
  }

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

      new ChangeLedID(m_robotContainer.getLedPanel(), 1).schedule();
      
  }

  public static Diagnostics getDiagnostics() {
    return diagnostics;
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
