// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;

public class Odometry extends SubsystemBase {
  private final PhotonCameraSubsystem photonVision;
  private final Drivetrain drivetrain;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d m_field = new Field2d();

  /* standard deviation of robot states, the lower the numbers arm, the more we trust odometry */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  
  /* standard deviation of vision readings, the lower the numbers arm, the more we trust vision */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private DriverStation.Alliance allianceColor = DriverStation.Alliance.Invalid;

  /** Creates a new Odometry. */
  public Odometry(PhotonCameraSubsystem photonVision, Drivetrain drivetrain) {
    this.photonVision = photonVision;
    this.drivetrain = drivetrain;
    
    SmartDashboard.putData("Field", m_field);

    poseEstimator =  new SwerveDrivePoseEstimator(
      drivetrain.getKinematics(),
            new Rotation2d(drivetrain.getNavxGyroValue()),
            new SwerveModulePosition[] {
              drivetrain.getM_frontLeft().getPosition(),
              drivetrain.getM_frontRight().getPosition(),
              drivetrain.getM_backLeft().getPosition(),
              drivetrain.getM_backRight().getPosition()
            },
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.logDouble("drivetrain/BR S", Math.toDegrees(drivetrain.getM_backRight().getSteerEncPosition()), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BL S", Math.toDegrees(drivetrain.getM_backLeft().getSteerEncPosition()), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/FR S", Math.toDegrees(drivetrain.getM_frontRight().getSteerEncPosition()), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BR S", Math.toDegrees(drivetrain.getM_backRight().getSteerEncPosition()), Constants.ENABLE_LOGGING);

    Logger.logDouble("drivetrain/BR D", drivetrain.getM_backRight().getDriveEncPosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BL D", drivetrain.getM_backLeft().getDriveEncPosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/FR D", drivetrain.getM_frontRight().getDriveEncPosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/FL D", drivetrain.getM_frontLeft().getDriveEncPosition(), Constants.ENABLE_LOGGING);

    if (Constants.DRIVETRAIN_DEBUG) {
      SmartShuffleboard.put("Drive", "distance to desired", 2 - poseEstimator.getEstimatedPosition().getX());
      SmartShuffleboard.put("Drive", "Abs Encoder", "FR abs", drivetrain.getFrontRightCanCoder().getAbsolutePosition());
      SmartShuffleboard.put("Drive", "Abs Encoder", "FL abs", drivetrain.getFrontLeftCanCoder().getAbsolutePosition());
      SmartShuffleboard.put("Drive", "Abs Encoder", "BR abs", drivetrain.getBackRightCanCoder().getAbsolutePosition());
      SmartShuffleboard.put("Drive", "Abs Encoder", "BL abs", drivetrain.getBackLeftCanCoder().getAbsolutePosition());

      SmartShuffleboard.put("Drive", "Steer Encoders", "BR S", Math.toDegrees(drivetrain.getM_backRight().getSteerEncPosition()));
      SmartShuffleboard.put("Drive", "Steer Encoders", "BL S", Math.toDegrees(drivetrain.getM_backLeft().getSteerEncPosition()));
      SmartShuffleboard.put("Drive", "Steer Encoders", "FR S", Math.toDegrees(drivetrain.getM_frontRight().getSteerEncPosition()));
      SmartShuffleboard.put("Drive", "Steer Encoders", "FL S", Math.toDegrees(drivetrain.getM_frontLeft().getSteerEncPosition()));

      SmartShuffleboard.put("Drive", "Drive Encoders", "BR D", drivetrain.getM_backRight().getDriveEncPosition());
      SmartShuffleboard.put("Drive", "Drive Encoders", "BL D", drivetrain.getM_backLeft().getDriveEncPosition());
      SmartShuffleboard.put("Drive", "Drive Encoders", "FR D", drivetrain.getM_frontRight().getDriveEncPosition());
      SmartShuffleboard.put("Drive", "Drive Encoders", "FL D", drivetrain.getM_frontLeft().getDriveEncPosition());

      SmartShuffleboard.put("Drive", "Odometry","odometry x", poseEstimator.getEstimatedPosition().getX());
      SmartShuffleboard.put("Drive", "Odometry","odometry y", poseEstimator.getEstimatedPosition().getY());
      SmartShuffleboard.put("Drive", "Odometry","odometry angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    if (DriverStation.isEnabled()) {
      poseEstimator.update((new Rotation2d(Math.toRadians(drivetrain.getNavxGyroValue()))),
              new SwerveModulePosition[]{
                      drivetrain.getM_frontLeft().getPosition(), drivetrain.getM_frontRight().getPosition(),
                      drivetrain.getM_backLeft().getPosition(), drivetrain.getM_backRight().getPosition()
              });
      Logger.logPose2d("/odometry/robot", poseEstimator.getEstimatedPosition(), Constants.ENABLE_LOGGING);
      if (Constants.ADD_VISION_TO_ODOMETRY && DriverStation.isTeleop() && photonVision != null) {
        Pose2d visionPose = photonVision.getRobot2dFieldPose();
        if (visionPose != null) {
          double latency = photonVision.getCameraLatencyMs();
          if ((latency > 0) && (latency < Constants.VISION_MAX_LATENCY)) {
            Logger.logBoolean("/odometry/addingVision", true,Constants.ENABLE_LOGGING);
            poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - latency);
            Logger.logBoolean("/odometry/addingVision", false,Constants.ENABLE_LOGGING);
          }
        }
      }
    }
    m_field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(new Rotation2d(Math.toRadians(drivetrain.getNavxGyroValue())),
            new SwerveModulePosition[]{
              drivetrain.getM_frontLeft().getPosition(), drivetrain.getM_frontRight().getPosition(),
              drivetrain.getM_backLeft().getPosition(), drivetrain.getM_backRight().getPosition()
            }, pose);
  }

  public SwerveDrivePoseEstimator getOdometry () {
    return poseEstimator;
  }

  public double getPoseX() {
    return poseEstimator.getEstimatedPosition().getX();
  }

  public double getPoseY() {
    return poseEstimator.getEstimatedPosition().getY();
  }
  public double getPoseAngleRad() {
    return poseEstimator.getEstimatedPosition().getRotation().getRadians();
  }

  public Field2d getField() {
    return m_field;
  }
}
