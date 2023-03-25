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
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagDutyCycleEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

public class GripperSubsystem extends SubsystemBase {
  public WPI_TalonSRX gripperMotor;
  private DutyCycleEncoder gripperEncoder;
  private ProtectionMechanism protectionMechanism;
  private boolean hasPiece = false;

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
    if (Constants.GRIPPER_DEBUG) {
      SmartShuffleboard.put("Gripper", "Encoder", gripperPosition());
      SmartShuffleboard.put("Gripper", "Fwd Limit", gripperMotor.isFwdLimitSwitchClosed()==1);
      SmartShuffleboard.put("Gripper", "rev Limit", gripperMotor.isRevLimitSwitchClosed()==1);
    }
    SmartShuffleboard.put("Driver", "HAS PIECE", hasPiece);

    Logger.logBoolean("/gripper/closedLimit", getClosedLimitSwitch(),Constants.ENABLE_LOGGING);
    Logger.logBoolean("/gripper/openLimit", getopenLimitSwitch(),Constants.ENABLE_LOGGING);
    Logger.logDouble("/gripper/encoder", gripperPosition(),Constants.ENABLE_LOGGING);
  }

  public void open() {
    gripperMotor.set(validateGripperVolt(Constants.GRIPPER_OPENING_SPEED));
  }

  public void close() {
    gripperMotor.set(Constants.GRIPPER_CLOSING_SPEED);
  }

  public void stop() {
    gripperMotor.set(0);
    // gripperMotor.stopMotor();
  }
  public void move(double speed) {
    gripperMotor.set(validateGripperVolt(speed));
//    gripperMotor.set(speed > 0 && Mechanism.getInstance().safeToOpenGripper() ? speed : 0);
  }
  public double gripperPosition() {
    return gripperEncoder.get();
  }
  public boolean getopenLimitSwitch() {
    return gripperMotor.isFwdLimitSwitchClosed() == 1;
  }
  public boolean getClosedLimitSwitch() {
    return gripperMotor.isRevLimitSwitchClosed() == 1;
  }
  public double getSpeed(){
    return gripperMotor.get();
  }

  public void setProtectionMechanism(ProtectionMechanism protectionMechanism) {
    this.protectionMechanism = protectionMechanism;
  }
  public double validateGripperVolt(double volt){
    if ((volt > 0 && protectionMechanism.safeToOpenGripper()) || volt < 0) return volt;
    return 0;
  }

  public void setHasPiece(boolean bool) {
    hasPiece = bool;
  }
  public boolean getHasPiece() {
    return hasPiece;
  }

}
