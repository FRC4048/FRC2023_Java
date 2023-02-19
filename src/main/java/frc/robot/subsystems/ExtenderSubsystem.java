// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ExtenderPosition;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagTalonSrxEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;
import frc.robot.Constants;


//import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class ExtenderSubsystem extends SubsystemBase {

  private WPI_TalonSRX extenderMotor;
  private double extenderSetPoint;
  private boolean isManualControl;

  public ExtenderSubsystem() {
    isManualControl = false;
    extenderMotor = new WPI_TalonSRX(Constants.EXTENDER_MOTOR_ID);

    //motor configs
    extenderMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.EXTENDER_MOTOR_TIMEOUT);
    extenderMotor.setNeutralMode(NeutralMode.Brake);
    extenderMotor.selectProfileSlot(0, 0);
    extenderMotor.configAllowableClosedloopError(0, Constants.EXTENDER_POSITION_ERROR, Constants.EXTENDER_MOTOR_TIMEOUT);
    extenderMotor.config_kP(0, Constants.EXTENDER_P, Constants.EXTENDER_MOTOR_TIMEOUT);
    extenderMotor.config_kI(0,Constants.EXTENDER_I, Constants.EXTENDER_MOTOR_TIMEOUT);
    extenderMotor.config_kD(0,Constants.EXTENDER_D , Constants.EXTENDER_MOTOR_TIMEOUT);
    extenderMotor.config_kF(0, Constants.EXTENDER_F, Constants.EXTENDER_MOTOR_TIMEOUT);

    //limit switch
    extenderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    extenderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    extenderMotor.setInverted(true); 
    extenderMotor.setSensorPhase(true); 

    resetEncoder();
    extenderSetPoint = getExtenderEncoder();
    
  }
  public void setManualMode(boolean manualMode ){
    isManualControl = manualMode;

  }
  public void setExtenderSpeed(double speed){
      extenderMotor.set(speed);
  }

  public void stopExtender(){
      extenderMotor.set(0);
  }

  public double getExtenderEncoder() {
    return extenderMotor.getSelectedSensorPosition();
  }

  private void moveExtender() {
    double position = extenderSetPoint;
    extenderMotor.set(ControlMode.Position, (int)position);
  }

  public void extendToPosition(ExtenderPosition elevatorPosition) {
    isManualControl=false;
    extenderSetPoint = elevatorPosition.getPosition();
  }

  public void extendToPositionManual(double value){
    isManualControl=true;
    extenderMotor.set(ControlMode.Position, getExtenderEncoder()+value);
  }

  public boolean extendAtPos(ExtenderPosition elevatorPosition) {
    return Math.abs(elevatorPosition.getPosition() - Math.abs(getExtenderEncoder())) < Constants.EXTENDER_POSITION_ERROR;
  }

  public double getError() {
    return extenderMotor.getClosedLoopError(0);
  }
  public void resetEncoder() {
    extenderMotor.setSelectedSensorPosition(0, 0, Constants.EXTENDER_MOTOR_TIMEOUT);
  }

  public boolean getTopSwitch() {
    return extenderMotor.getSensorCollection().isRevLimitSwitchClosed();
  }
  public boolean getBottomSwitch(){
    return extenderMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    if(!isManualControl){
      moveExtender();
    }
  
   SmartShuffleboard.put("test", "encoder", getExtenderEncoder());

  }
}
