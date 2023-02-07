// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ExtenderPosition;
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
  //public static final double EXTENDER_ENCODER_SCALE = 256/20;
  private final double EXTENDER_UP_SCALE_FACTOR = 1.0;
  private final double EXTENDER_DOWN_SCALE_FACTOR = 1.0;
  //private WPI_TalonSRX extenderMotor2;
  //private AnalogPotentiometer potentiometer;

  public ExtenderSubsystem() {
    isManualControl = false;
    extenderMotor = new WPI_TalonSRX(0);

    //motor configs
    extenderMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.EXTENDER_MOTOR_TIMEOUT);
    //extenderMotor.configNominalOutputForward(0, Constants.EXTENDER_MOTOR_TIMEOUT);   -> might be needed later
    //extenderMotor.configNominalOutputReverse(0, Constants.EXTENDER_MOTOR_TIMEOUT);
    //extenderMotor.configPeakOutputForward(EXTENDER_UP_SCALE_FACTOR, Constants.EXTENDER_MOTOR_TIMEOUT);
    //extenderMotor.configPeakOutputReverse(-EXTENDER_DOWN_SCALE_FACTOR, Constants.EXTENDER_MOTOR_TIMEOUT);
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

    resetEncoder();
    extenderSetPoint = getExtenderEncoder();
    //extenderMotor2 = new WPI_TalonSRX(1);
    //extenderMotor2.follow(extenderMotor);
    //potentiometer = new AnalogPotentiometer(Constants.EXTENDER_POTENTIOMETER, Constants.EXTENDER_RANGE_OF_MOTION,Constants.EXTENDER_STARTING_POINT);
  }
  public void setManualMode(boolean manualMode ){
    isManualControl = manualMode;

  }
  public void setExtenderSpeed(double speed){
      extenderMotor.set(speed);
  }

  public void stopExtender(){
      extenderMotor.stopMotor();
  }

  public double getExtenderEncoder() {
    return extenderMotor.getSelectedSensorPosition();
  }

  private void moveExtender() {
    double position = extenderSetPoint;
    extenderMotor.set(ControlMode.Position, (int)position);
  }

  public void extendToPosition(ExtenderPosition elevatorPosition) {
    extenderSetPoint = elevatorPosition.getPosition();
  }

  public boolean extendAtPos(ExtenderPosition elevatorPosition) {
    return Math.abs(elevatorPosition.getPosition() - getExtenderEncoder()) < Constants.EXTENDER_POSITION_ERROR*2;
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
  }
}
