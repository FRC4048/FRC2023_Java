// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.drive.WheelAlign;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.Diagnostics;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer m_robotContainer;
  private static Diagnostics diagnostics;
  private double loopTime = 0;
  private DoubleArrayPublisher gyro;
  private DoubleArraySubscriber localizationResult;
  private BooleanPublisher matchStartPublisher;
  private boolean started = false;

  @Override
  public void robotInit() {
    if (Constants.ENABLE_LOGGING) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog(), true);
    }
    diagnostics = new Diagnostics();
    m_robotContainer = new RobotContainer();
    new WheelAlign(m_robotContainer.getDrivetrain()).schedule();
    new ResetGyro(m_robotContainer.getDrivetrain(), 2).schedule();
    new ResetOdometry(m_robotContainer.getDrivetrain(), 0, 13.5, Math.toRadians(180), 3).schedule();
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shuffleboard/Test");
    gyro = table.getDoubleArrayTopic("Gyro").publish();
    matchStartPublisher = table.getBooleanTopic("matchStartPublisher").publish();
    localizationResult = table.getDoubleArrayTopic("LocalizationResult").subscribe(new double[]{-1,-1,-1});
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (DriverStation.isEnabled()){
      gyro.set(new double[] {m_robotContainer.getDrivetrain().getPoseX()*-1,m_robotContainer.getDrivetrain().getPoseY()*-1, m_robotContainer.getDrivetrain().getPoseAngleRad()});
    }
    matchStartPublisher.set(started);
    // Logger should stay at the end of robotPeriodic()
    double time = (loopTime == 0) ? 0 : (Timer.getFPGATimestamp() - loopTime) * 1000;
    Logger.logDouble("/robot/loopTime", time, Constants.ENABLE_LOGGING);
    Pose2d amclPose = new Pose2d(localizationResult.get()[0], localizationResult.get()[1], new Rotation2d(localizationResult.get()[2]));
    Logger.logPose2d("/odometry/amclResult",amclPose, Constants.ENABLE_LOGGING);
  }


  @Override
  public void disabledInit() {
    m_robotContainer.getDisabledLedCycleCommand().initialize();
    started = false;
  }

  @Override
  public void disabledPeriodic() {
    loopTime = 0;
    SmartShuffleboard.put("Autonomous", "Chosen Action, Location",
            m_robotContainer.getAutonomousChooser().getAction().name() + ", " + m_robotContainer.getAutonomousChooser().getLocation().name()).withPosition(0,2).withSize(4,1);
    m_robotContainer.getDisabledLedCycleCommand().refresh();
    matchStartPublisher.set(started);
  }

  @Override
  public void autonomousInit() {
    started = true;
    autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    loopTime = Timer.getFPGATimestamp();
    m_robotContainer.getAutoLedCycleCommand().refresh();
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
    loopTime = Timer.getFPGATimestamp();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    diagnostics.reset();
    m_robotContainer.getArm().zeroPID();
    m_robotContainer.getTestLedCycleCommand().initialize();
  }

  @Override
  public void testPeriodic() {
    loopTime = 0;
    m_robotContainer.getTestLedCycleCommand().refresh();
    diagnostics.refresh();
//    TrajectoryConfig config =
//      new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION).setKinematics(m_robotContainer.getDrivetrain().getKinematics());
//
//    Trajectory testTrajectory =
//      TrajectoryGenerator.generateTrajectory(
//        new Pose2d(0, 0, new Rotation2d(0)),
//        List.of(new Translation2d(1, 0)),
//        new Pose2d(2, 0, new Rotation2d(0)),
//        config);
//        double time = testTrajectory.getTotalTimeSeconds();
  }

  public static Diagnostics getDiagnostics() {
    return diagnostics;
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    loopTime = Timer.getFPGATimestamp();
  }
}
