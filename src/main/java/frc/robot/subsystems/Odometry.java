// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;

public class Odometry extends SubsystemBase {
  /** Creates a new Odometry. */
  public Odometry(PhotonCameraSubsystem photonVision) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.logDouble("drivetrain/FR abs", frontRightCanCoder.getAbsolutePosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/FL abs", frontLeftCanCoder.getAbsolutePosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BR abs", backRightCanCoder.getAbsolutePosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BL abs", backLeftCanCoder.getAbsolutePosition(), Constants.ENABLE_LOGGING);

    Logger.logDouble("drivetrain/BR S", Math.toDegrees(m_backRight.getSteerEncPosition()), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BL S", Math.toDegrees(m_backLeft.getSteerEncPosition()), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/FR S", Math.toDegrees(m_frontRight.getSteerEncPosition()), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BR S", Math.toDegrees(m_backRight.getSteerEncPosition()), Constants.ENABLE_LOGGING);

    Logger.logDouble("drivetrain/BR D", m_backRight.getDriveEncPosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/BL D", m_backLeft.getDriveEncPosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/FR D", m_frontRight.getDriveEncPosition(), Constants.ENABLE_LOGGING);
    Logger.logDouble("drivetrain/FL D", m_frontLeft.getDriveEncPosition(), Constants.ENABLE_LOGGING);
    filterRoll = (float)rollFilter.calculate((double)getRoll());

    if (Constants.DRIVETRAIN_DEBUG) {
      SmartShuffleboard.put("Drive", "FilterRoll", filterRoll);
      SmartShuffleboard.put("Drive", "distance to desired", 2 - poseEstimator.getEstimatedPosition().getX());
      SmartShuffleboard.put("Drive", "Abs Encoder", "FR abs", frontRightCanCoder.getAbsolutePosition());
      SmartShuffleboard.put("Drive", "Abs Encoder", "FL abs", frontLeftCanCoder.getAbsolutePosition());
      SmartShuffleboard.put("Drive", "Abs Encoder", "BR abs", backRightCanCoder.getAbsolutePosition());
      SmartShuffleboard.put("Drive", "Abs Encoder", "BL abs", backLeftCanCoder.getAbsolutePosition());

      SmartShuffleboard.put("Drive", "Steer Encoders", "BR S", Math.toDegrees(m_backRight.getSteerEncPosition()));
      SmartShuffleboard.put("Drive", "Steer Encoders", "BL S", Math.toDegrees(m_backLeft.getSteerEncPosition()));
      SmartShuffleboard.put("Drive", "Steer Encoders", "FR S", Math.toDegrees(m_frontRight.getSteerEncPosition()));
      SmartShuffleboard.put("Drive", "Steer Encoders", "FL S", Math.toDegrees(m_frontLeft.getSteerEncPosition()));

      SmartShuffleboard.put("Drive", "Drive Encoders", "BR D", m_backRight.getDriveEncPosition());
      SmartShuffleboard.put("Drive", "Drive Encoders", "BL D", m_backLeft.getDriveEncPosition());
      SmartShuffleboard.put("Drive", "Drive Encoders", "FR D", m_frontRight.getDriveEncPosition());
      SmartShuffleboard.put("Drive", "Drive Encoders", "FL D", m_frontLeft.getDriveEncPosition());

      SmartShuffleboard.put("Drive", "Odometry","odometry x", poseEstimator.getEstimatedPosition().getX());
      SmartShuffleboard.put("Drive", "Odometry","odometry y", poseEstimator.getEstimatedPosition().getY());
      SmartShuffleboard.put("Drive", "Odometry","odometry angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    if (DriverStation.isEnabled()) {
      poseEstimator.update((new Rotation2d(Math.toRadians(getNavxGyroValue()))),
              new SwerveModulePosition[]{
                      m_frontLeft.getPosition(), m_frontRight.getPosition(),
                      m_backLeft.getPosition(), m_backRight.getPosition()
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
    /* if Red alliance, mirror pose on field */
    if (allianceColor == DriverStation.Alliance.Red) {
      m_field.setRobotPose(new Pose2d(
              Units.feetToMeters(Constants.FIELD_LENGTH_X_FEET) - poseEstimator.getEstimatedPosition().getX(),
              Units.feetToMeters(Constants.FIELD_LENGTH_Y_FEET) - poseEstimator.getEstimatedPosition().getY(),
              new Rotation2d(poseEstimator.getEstimatedPosition().getRotation().getRadians()+Math.PI)));
    } else {
      m_field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(new Rotation2d(Math.toRadians(getNavxGyroValue())),
            new SwerveModulePosition[]{
                    m_frontLeft.getPosition(), m_frontRight.getPosition(),
                    m_backLeft.getPosition(), m_backRight.getPosition()
            }, pose);
  }
}
