// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagDutyCycleEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

public class GripperSubsystem extends SubsystemBase {
  public WPI_TalonSRX gripperMotor;
  private DutyCycleEncoder gripperEncoder;

  public GripperSubsystem() {
    gripperMotor = new WPI_TalonSRX(Constants.GRIPPER_MOTOR_ID);
    gripperEncoder = new DutyCycleEncoder(Constants.GRIPPER_ENCODER_ID);
    gripperMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    gripperMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    Robot.getDiagnostics().addDiagnosable(new DiagDutyCycleEncoder("Gripper", "Encoder", 10, gripperEncoder));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Gripper", "Open Switch", gripperMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Gripper", "Close Switch", gripperMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));

  }
 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartShuffleboard.put("Gripper", "Encoder", gripperPosition());

    SmartShuffleboard.put("Gripper", "Limit Switches", "Fwd Limit", gripperMotor.isFwdLimitSwitchClosed());
    SmartShuffleboard.put("Gripper", "Limit Switches", "rev Limit", gripperMotor.isRevLimitSwitchClosed());

  }

  public void open() {
    gripperMotor.set(ProtectionMechanism.getInstance().validateGripperVolt(Constants.GRIPPER_OPENING_SPEED));
//    gripperMotor.set(Mechanism.getInstance().safeToOpenGripper()?Constants.GRIPPER_OPENING_SPEED:0);
  }

  public void close() {
    gripperMotor.set(Constants.GRIPPER_CLOSING_SPEED);
  }

  public void stop() {
    gripperMotor.set(0);
    // gripperMotor.stopMotor();
  }
  public void move(double speed) {
    gripperMotor.set(ProtectionMechanism.getInstance().validateGripperVolt(speed));
//    gripperMotor.set(speed > 0 && Mechanism.getInstance().safeToOpenGripper() ? speed : 0);
  }
  public double gripperPosition() {
    return gripperEncoder.get();
  }
  public boolean getopenLimitSwitch() {
    return gripperMotor.isFwdLimitSwitchClosed() == 1;
  }
  public double getSpeed(){
    return gripperMotor.get();
  }
}
