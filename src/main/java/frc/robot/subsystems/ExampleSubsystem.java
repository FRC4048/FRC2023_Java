// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class ExampleSubsystem extends SubsystemBase {
 
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double kVoltage;
  private double encoderValue;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private int deviceID;
  
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    kVoltage = 0.0;
    deviceID = 45;
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    m_motor.restoreFactoryDefaults();

    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    encoderValue = m_encoder.getPosition();

    // PID coefficients
    kP = 1; 
    kI = 0;
    kD = 0;


    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);

    
    //SmartShuffleboard.put("PID", "Voltage", 0);
    SmartShuffleboard.put("PID", "P Gain", kP);
    SmartShuffleboard.put("PID", "I Gain", kI);
    SmartShuffleboard.put("PID", "D Gain", kD);
  }


  @Override
  public void periodic() {

    System.out.println(encoderValue);

    //double voltage = SmartShuffleboard.getDouble("PID", "Voltage", 0);
    double p = SmartShuffleboard.getDouble("PID", "P Gain", 1);
    double i = SmartShuffleboard.getDouble("PID", "I Gain", 0);
    double d = SmartShuffleboard.getDouble("PID", "D Gain", 0);

    // if((voltage != kVoltage)) {
    //   kVoltage = voltage;
    //   m_motor.set(voltage);
    // }
    if((p != kP)) { 
      m_pidController.setP(p); 
      kP = p; 
    }
    if((i != kI)) { 
      m_pidController.setI(i); 
      kI = i; 
    }
    if((d != kD)) { 
      m_pidController.setD(d); 
      kD = d; 
    }
    m_pidController.setReference(0, ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setVoltage(double voltage) {
    m_motor.set(voltage);
  }
}
