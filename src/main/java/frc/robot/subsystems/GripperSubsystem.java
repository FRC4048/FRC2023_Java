// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
  public WPI_TalonSRX gripperMotor = new WPI_TalonSRX(0);
  private DutyCycleEncoder gripperEncoder = new DutyCycleEncoder(2);
  public GripperSubsystem() {
    gripperMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    gripperMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }
 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open(double speed) {
    gripperMotor.set(speed);
  }

  public void close(double speed) {
    gripperMotor.set(-1 * speed);
  }

  public void stop() {
    gripperMotor.stopMotor();
  }
  
  public double gripperPosition() {
    return gripperEncoder.getAbsolutePosition();
  }
  public boolean getopenLimitSwitch() {
    return true;
  }
}