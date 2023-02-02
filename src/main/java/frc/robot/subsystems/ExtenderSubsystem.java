// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.utils.diag.DiagTalonSrxEncoder;
//import frc.robot.utils.diag.DiagTalonSrxSwitch;
import frc.robot.Constants;
//import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;


public class ExtenderSubsystem extends SubsystemBase {

  private WPI_TalonSRX extenderMotor;  
  private AnalogPotentiometer potentiometer;

  public ExtenderSubsystem() {
    extenderMotor = new WPI_TalonSRX(Constants.EXTENDER_MOTOR_ID);
    potentiometer = new AnalogPotentiometer(Constants.EXTENDER_POTENTIOMETER, Constants.EXTENDER_RANGE_OF_MOTION,Constants.EXTENDER_STARTING_POINT);
  
    extenderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    extenderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }

 public void setEntenderSpeed(double speed){
    extenderMotor.set(speed);
 }

 public void stopExtender(){
    extenderMotor.set(0);
 }

 public double getExtenderEncoder() {
  return extenderMotor.getSelectedSensorPosition();
}

 public boolean getTopSwitch() {
  return extenderMotor.getSensorCollection().isRevLimitSwitchClosed();
}

public boolean getBottomSwitch(){
  return extenderMotor.getSensorCollection().isFwdLimitSwitchClosed();
}

public double getPotentiometer() {
  return potentiometer.get();
}

  @Override
  public void periodic() {}
}
