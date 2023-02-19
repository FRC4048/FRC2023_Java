// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
  public WPI_TalonSRX gripperMotor = new WPI_TalonSRX(Constants.GRIPPER_MOTOR_ID);
  private DutyCycleEncoder gripperEncoder = new DutyCycleEncoder(Constants.GRIPPER_ENCODER_ID);
  public GripperSubsystem() {
   // gripperMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
   // gripperMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }
 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartShuffleboard.put("Gripper", "Encoder", gripperPosition());
  }

  public void open() {
    gripperMotor.set(Constants.GRIPPER_OPENING_SPEED);
  }

  public void close() {
    gripperMotor.set(Constants.GRIPPER_CLOSING_SPEED);
  }

  public void stop() {
    gripperMotor.stopMotor();
  }
  
  public double gripperPosition() {
    return gripperEncoder.get();
  }
  public boolean getopenLimitSwitch() {
    return true;
  }
}