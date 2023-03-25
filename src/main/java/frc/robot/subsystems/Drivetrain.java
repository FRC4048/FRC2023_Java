// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxAbsEncoder;
import frc.robot.utils.diag.DiagSparkMaxEncoder;

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
  private ShuffleboardTab driverTab;
  private GenericEntry gyroEntry;
  private float filterRoll = 0;

  private final AHRS navxGyro;
  private double navxGyroValue;

  private final Field2d m_field = new Field2d();
  private DriverStation.Alliance allianceColor = DriverStation.Alliance.Invalid;

  private final MedianFilter rollFilter;

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDrivePoseEstimator poseEstimator;

  private final PhotonCameraSubsystem photonVision;

  /* standard deviation of robot states, the lower the numbers arm, the more we trust odometry */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /* standard deviation of vision readings, the lower the numbers arm, the more we trust vision */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);



  public Drivetrain(PhotonCameraSubsystem photonVision) {
    this.photonVision = photonVision;
    navxGyro = new AHRS();

    navxGyroValue = -1;

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

    driverTab = Shuffleboard.getTab("Driver");
    gyroEntry = driverTab.add("Gyro Value", 0).withPosition(5, 0).withWidget("Gyro").withSize(2, 4).getEntry();

    rollFilter = new MedianFilter(5);

    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Drive", "Front Left", Constants.DIAG_REL_SPARK_ENCODER, m_frontLeftDrive));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Drive", "Front Right", Constants.DIAG_REL_SPARK_ENCODER, m_frontRightDrive));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Drive", "Back Left", Constants.DIAG_REL_SPARK_ENCODER, m_backLeftDrive));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Drive", "Back Right", Constants.DIAG_REL_SPARK_ENCODER, m_backRightDrive));
   
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Turn", "Front Left", Constants.DIAG_REL_SPARK_ENCODER, m_frontLeftTurn));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Turn", "Front Right", Constants.DIAG_REL_SPARK_ENCODER, m_frontRightTurn));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Turn", "Back Left", Constants.DIAG_REL_SPARK_ENCODER, m_backLeftTurn));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("DT Turn", "Back Right", Constants.DIAG_REL_SPARK_ENCODER, m_backRightTurn));
  
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxAbsEncoder("DT CanCoder", "Front Left", Constants.DIAG_ABS_SPARK_ENCODER, frontLeftCanCoder));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxAbsEncoder("DT CanCoder", "Front Right", Constants.DIAG_ABS_SPARK_ENCODER, frontRightCanCoder));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxAbsEncoder("DT CanCoder", "Back Left", Constants.DIAG_ABS_SPARK_ENCODER, backLeftCanCoder));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxAbsEncoder("DT CanCoder", "Back Right", Constants.DIAG_ABS_SPARK_ENCODER, backRightCanCoder));


    m_frontLeft = new SwerveModule(m_frontLeftDrive, m_frontLeftTurn, frontLeftCanCoder, 1);
    m_frontRight = new SwerveModule(m_frontRightDrive, m_frontRightTurn, frontRightCanCoder, 2);
    m_backLeft = new SwerveModule(m_backLeftDrive, m_backLeftTurn, backLeftCanCoder, 3);
    m_backRight = new SwerveModule(m_backRightDrive, m_backRightTurn, backRightCanCoder, 4);

    m_frontLeftDrive.setInverted(true);
    m_frontRightDrive.setInverted(false);
    m_backRightDrive.setInverted(false);
    m_backLeftDrive.setInverted(true);

    poseEstimator =  new SwerveDrivePoseEstimator(
            m_kinematics,
            new Rotation2d(getNavxGyroValue()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            },
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);

    setGyroOffset(180);
  }

  public double getNavxGyroValue() {
    return navxGyroValue;
  }

  private double getGyro() {
    return (navxGyro.getAngle() % 360)*-1; //ccw should be positive
  }

  public float getRoll() {
    return navxGyro.getRoll();
  }

  public float getFilterRoll() {
    return filterRoll;
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(Math.toRadians(getNavxGyroValue())))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_VELOCITY);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }


  public void stopMotors() {
    m_backRightDrive.set(0.0);
    m_backLeftDrive.set(0.0);
    m_frontRightDrive.set(0.0);
    m_frontLeftDrive.set(0.0);
    m_backRightTurn.set(0.0);
    m_backLeftTurn.set(0.0);
    m_frontRightTurn.set(0.0);
    m_frontLeftTurn.set(0.0);
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
        default: return CAN;
    }
  }

  @Override
  public void periodic() {
    navxGyroValue = getGyro();
    gyroEntry.setDouble(getNavxGyroValue());
    Logger.logDouble("/drivetrain/gyro", navxGyroValue, Constants.ENABLE_LOGGING);
    filterRoll = (float)rollFilter.calculate((double)getRoll());

    SmartShuffleboard.put("Auto Balance", "FilterRoll", filterRoll);

    if (Constants.DRIVETRAIN_DEBUG) {
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

  public SwerveDriveKinematics getKinematics () {
    return m_kinematics;
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

  public void setGyroOffset(double offset) {
    gyroOffset = offset;
    navxGyro.setAngleAdjustment(gyroOffset);
    //navxGyro.getFusedHeading();
  }
  
  public double getGyroOffset() {
    return gyroOffset;
  }

  public Field2d getField() {
    return m_field;
  }

  public void setAllianceColor(DriverStation.Alliance color) {
    allianceColor = color;
  }
}