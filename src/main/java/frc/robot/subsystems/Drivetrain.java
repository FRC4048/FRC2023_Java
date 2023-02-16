// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase{

  private final CANSparkMax m_frontLeftDrive;
  private final CANSparkMax m_frontRightDrive;
  private final CANSparkMax m_backLeftDrive;
  private final CANSparkMax m_backRightDrive;
  private final CANSparkMax m_frontLeftTurn;
  private final CANSparkMax m_frontRightTurn;
  private final CANSparkMax m_backLeftTurn;
  private final CANSparkMax m_backRightTurn;

  private final WPI_CANCoder frontLeftCanCoder;
  private final WPI_CANCoder frontRightCanCoder;
  private final WPI_CANCoder backLeftCanCoder;
  private final WPI_CANCoder backRightCanCoder;

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.ROBOT_LENGTH/2, Constants.ROBOT_WIDTH/2);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.ROBOT_LENGTH/2, -Constants.ROBOT_WIDTH/2);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.ROBOT_LENGTH/2, Constants.ROBOT_WIDTH/2);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.ROBOT_LENGTH/2, -Constants.ROBOT_WIDTH/2);

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private double gyroOffset = 0;

  private final AHRS navxGyro;

  private final Field2d m_field = new Field2d();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry;
   

  public Drivetrain() {
    navxGyro = new AHRS();
    navxGyro.reset();

    SmartDashboard.putData("Field", m_field);
    
    m_frontLeftDrive = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_D, MotorType.kBrushless);
    m_frontRightDrive = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_D, MotorType.kBrushless);
    m_backLeftDrive = new CANSparkMax(Constants.DRIVE_BACK_LEFT_D, MotorType.kBrushless);
    m_backRightDrive = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_D, MotorType.kBrushless);

    m_frontLeftTurn = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_S, MotorType.kBrushless);
    m_frontRightTurn = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_S, MotorType.kBrushless);
    m_backLeftTurn = new CANSparkMax(Constants.DRIVE_BACK_LEFT_S, MotorType.kBrushless);
    m_backRightTurn = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_S, MotorType.kBrushless);

    frontLeftCanCoder = new WPI_CANCoder(Constants.DRIVE_CANCODER_FRONT_LEFT);
    frontRightCanCoder = new WPI_CANCoder(Constants.DRIVE_CANCODER_FRONT_RIGHT);
    backLeftCanCoder = new WPI_CANCoder(Constants.DRIVE_CANCODER_BACK_LEFT);
    backRightCanCoder = new WPI_CANCoder(Constants.DRIVE_CANCODER_BACK_RIGHT);

    m_frontLeft = new SwerveModule(m_frontLeftDrive, m_frontLeftTurn, frontLeftCanCoder, 1);
    m_frontRight = new SwerveModule(m_frontRightDrive, m_frontRightTurn, frontRightCanCoder, 2);
    m_backLeft = new SwerveModule(m_backLeftDrive, m_backLeftTurn, backLeftCanCoder, 3);
    m_backRight = new SwerveModule(m_backRightDrive, m_backRightTurn, backRightCanCoder, 4);

    m_frontLeftDrive.setInverted(true);
    m_frontRightDrive.setInverted(false);
    m_backRightDrive.setInverted(false);
    m_backLeftDrive.setInverted(true);
    
    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        new Rotation2d(getGyro()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(13.5), new Rotation2d()));
  }

  public double getGyro() {
    return (navxGyro.getAngle() % 360)*-1; //ccw should be positive
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
              fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navxGyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setPower(int motorID, double value){
    switch(motorID) {
        case Constants.DRIVE_BACK_RIGHT_D:
            m_backRightDrive.set(value);
            break;
        case Constants.DRIVE_BACK_LEFT_D:
            m_backLeftDrive.set(value);
            break;
        case Constants.DRIVE_FRONT_RIGHT_D:
            m_frontRightDrive.set(value);
            break;
        case Constants.DRIVE_FRONT_LEFT_D:
            m_frontLeftDrive.set(value);
            break;
            
        case Constants.DRIVE_BACK_RIGHT_S:
            m_backRightTurn.set(value);
            break;
        case Constants.DRIVE_BACK_LEFT_S:
            m_backLeftTurn.set(value);
            break;
        case Constants.DRIVE_FRONT_RIGHT_S:
            m_frontRightTurn.set(value);
            break;
        case Constants.DRIVE_FRONT_LEFT_S:
            m_frontLeftTurn.set(value);
            break;
    }
  }

  public void SetRelEncZero(){
    m_backLeft.ResetRelEnc();
    m_backRight.ResetRelEnc();
    m_frontLeft.ResetRelEnc();
    m_frontRight.ResetRelEnc();
  }  

  public void resetGyro(){
    navxGyro.reset();
  }

  public double getRelEnc(int CAN){
    switch (CAN){
      case Constants.DRIVE_BACK_RIGHT_D:
          return m_backRight.getDriveEncPosition();
        case Constants.DRIVE_BACK_LEFT_D:
          return m_backLeft.getDriveEncPosition();
        case Constants.DRIVE_FRONT_RIGHT_D:
          return m_frontRight.getDriveEncPosition();          
        case Constants.DRIVE_FRONT_LEFT_D:
          return m_frontLeft.getDriveEncPosition();
            
        // case Constants.DRIVE_BACK_RIGHT_S:
        //     m_backRightTurn.set(value);
        //     break;
        // case Constants.DRIVE_BACK_LEFT_S:
        //     m_backLeftTurn.set(value);
        //     break;
        // case Constants.DRIVE_FRONT_RIGHT_S:
        //     m_frontRightTurn.set(value);
        //     break;
        // case Constants.DRIVE_FRONT_LEFT_S:
        //     m_frontLeftTurn.set(value);=
        //     break;
        default: return CAN;
    }
  }

  @Override
  public void periodic(){
    SmartShuffleboard.put("Diag", "Abs Encoder", "FR", frontRightCanCoder.getAbsolutePosition());
    SmartShuffleboard.put("Diag", "Abs Encoder", "FL", frontLeftCanCoder.getAbsolutePosition());
    SmartShuffleboard.put("Diag", "Abs Encoder", "BR", backRightCanCoder.getAbsolutePosition());
    SmartShuffleboard.put("Diag", "Abs Encoder", "BL", backLeftCanCoder.getAbsolutePosition());

    SmartShuffleboard.put("Diag", "Gyro", getGyro());
    SmartShuffleboard.put("Drive", "Steer Encoders", "Back Right S", m_backRight.getSteerEncPosition());
    SmartShuffleboard.put("Drive", "Steer Encoders", "Back Left S", m_backLeft.getSteerEncPosition());
    SmartShuffleboard.put("Drive", "Steer Encoders", "Front Right S", m_frontRight.getSteerEncPosition());
    SmartShuffleboard.put("Drive", "Steer Encoders", "Front Left S", m_frontLeft.getSteerEncPosition());

    SmartShuffleboard.put("Drive", "Drive Encoders", "Back Right D", m_backRight.getDriveEncPosition());
    SmartShuffleboard.put("Drive", "Drive Encoders", "Back Left D", m_backLeft.getDriveEncPosition());
    SmartShuffleboard.put("Drive", "Drive Encoders", "Front Right D", m_frontRight.getDriveEncPosition());
    SmartShuffleboard.put("Drive", "Drive Encoders", "Front Left D", m_frontLeft.getDriveEncPosition());

    SmartShuffleboard.put("Diag", "Offset", getGyroOffset());
    SmartShuffleboard.put("Diag", "Gyro Adjust", navxGyro.getAngleAdjustment());
    SmartShuffleboard.put("Diag", "Rotate Adjust", navxGyro.getRotation2d().getDegrees());


    m_odometry.update(new Rotation2d(Math.toRadians(getGyro())),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_backLeft.getPosition(), m_backRight.getPosition()
    });
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }
 
  public CANSparkMax getM_frontLeftTurn() {
    return m_frontLeftTurn;
  }

  public CANSparkMax getM_frontRightTurn() {
    return m_frontRightTurn;
  }

  public CANSparkMax getM_backLeftTurn() {
    return m_backLeftTurn;
  }

  public CANSparkMax getM_backRightTurn() {
    return m_backRightTurn;
  }

  public SwerveModule getM_frontLeft() {
    return m_frontLeft;
  }

  public SwerveModule getM_frontRight() {
    return m_frontRight;
  }

  public SwerveModule getM_backLeft() {
    return m_backLeft;
  }

  public SwerveModule getM_backRight() {
    return m_backRight;
  }

  public WPI_CANCoder getFrontLeftCanCoder() {
    return frontLeftCanCoder;
  }

  public WPI_CANCoder getFrontRightCanCoder() {
    return frontRightCanCoder;
  }

  public WPI_CANCoder getBackLeftCanCoder() {
    return backLeftCanCoder;
  }

  public WPI_CANCoder getBackRightCanCoder() {
    return backRightCanCoder;
  }

  public void setGyroOffset(double offset) {
    gyroOffset = offset;
    navxGyro.setAngleAdjustment(gyroOffset);
  }
  public double getGyroOffset() {
    return gyroOffset;
  }
}