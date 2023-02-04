// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.commands.Drive;
import frc.robot.utils.SmartShuffleboard;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = Constants.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax driveMotor;
  private final CANSparkMax steerMotor;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder steerEncoder;
  private final WPI_CANCoder absEncoder;

  // TODO: Adjust gains
  private final PIDController m_drivePIDController =
     new PIDController(
          0.25, 
          0, 
          0);

  // TODO: Adjust gains
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          4,
          0.0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // TODO: Adjust gains
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.015, 0.285);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.2, 0.8);

  private int id;
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotor
   * @param turningMotor
   * @param absEncoder 
   * */

  public SwerveModule(
      CANSparkMax driveMotor,
      CANSparkMax steerMotor,
      WPI_CANCoder absEncoder,
      int id) {
    this.id = id; 
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.absEncoder = absEncoder;

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    driveMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setIdleMode(IdleMode.kBrake);
    
    //Does not work
    //driveMotor.burnFlash();
    //steerMotor.burnFlash();

    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();

    steerEncoder.setPosition(0);
    driveEncoder.setPosition(0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    driveEncoder.setVelocityConversionFactor((2 * Constants.WHEEL_RADIUS * Math.PI) / (Constants.CHASSIS_DRIVE_GEAR_RATIO * 60));
        
    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    steerEncoder.setPositionConversionFactor(2 * Math.PI / Constants.CHASSIS_STEER_GEAR_RATIO); //change this maybe?


    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(0, Math.PI * 2);


  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(steerEncoder.getPosition()));
  }

    /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(steerEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  // public SwerveModulePosition getPosition() {
  //   return new SwerveModulePosition(
  //       m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
  // }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerEncPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

    if (id == 1) {
    SmartShuffleboard.put("Drive", "CSpeed" + id, driveEncoder.getVelocity());
    SmartShuffleboard.put("Drive", "DSpeed" + id, state.speedMetersPerSecond);
  }

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getSteerEncPosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);


        if (id == 1) {
          SmartShuffleboard.put("Drive", "CPos" + id, getSteerEncPosition());
          SmartShuffleboard.put("Drive", "DPos" + id, state.angle.getRadians());
        }
    driveMotor.set(driveOutput + driveFeedforward); 
    steerMotor.set(turnOutput + turnFeedforward);

    if (id == 1) {
    //SmartShuffleboard.put("Drive", "Feed forward" + id, driveFeedforward);
    //SmartShuffleboard.put("Drive", "Drive Output" + id, driveOutput);
    SmartShuffleboard.put("Drive", "Steer Feed forward" + id, turnFeedforward);
    SmartShuffleboard.put("Drive", "Steer Output" + id, turnOutput);
  }
  }
  public void ResetRelEnc() {
    steerEncoder.setPosition(0);
    driveEncoder.setPosition(0);
  }
  public void goToAbsEnc(double encZero){
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(Math.toRadians(absEncoder.getAbsolutePosition()), Math.toRadians(encZero));

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    steerMotor.setVoltage(turnOutput + turnFeedforward);    
  }

  public double getDriveEncPosition(){
    return driveEncoder.getPosition();
  }

  public double getSteerEncPosition(){
    double value = steerEncoder.getPosition();
    value %= 2 * Math.PI;
    if (value < 0) {
      value += 2 * Math.PI;
    } 
    return (value);
  }
}