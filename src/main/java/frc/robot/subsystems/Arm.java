package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public class Arm extends SubsystemBase {
  private double angle;
  private CANSparkMax neoMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private double encoderValue;
  public double kP, kI, kD, kIz, kFF, kVoltage;
  private boolean goingUp;
  
  public Arm() {
    angle = 0;

    neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
    encoder = neoMotor.getEncoder();

    neoMotor.restoreFactoryDefaults();
    neoMotor.setIdleMode(IdleMode.kBrake);
    encoder.setPosition(0);

    pidController = neoMotor.getPIDController();
    

    kP = Constants.ARM_PID_P_UP;
    //kI = Constants.ARM_PID_I;
    kD = Constants.ARM_PID_D;
    kFF = Constants.ARM_PID_FF;
  
    pidController.setP(kP);
    //pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kFF);
    
    SmartShuffleboard.put("PID", "P Gain", kP);
    SmartShuffleboard.put("PID", "I Gain", kI);
    SmartShuffleboard.put("PID", "D Gain", kD);
    SmartShuffleboard.put("PID", "FF Gain", kFF);
  }

  @Override
  public void periodic() {
    encoderValue = encoder.getPosition();
    
    /* 
    double p = SmartShuffleboard.getDouble("PID", "P Gain", Constants.ARM_PID_P);
    double i = SmartShuffleboard.getDouble("PID", "I Gain", Constants.ARM_PID_I);
    double d = SmartShuffleboard.getDouble("PID", "D Gain", Constants.ARM_PID_D);
    double ff = SmartShuffleboard.getDouble("PID", "FF Gain", Constants.ARM_PID_FF);
    */
    SmartShuffleboard.put("PID", "going up", goingUp);
    if (goingUp) {
      pidController.setP(Constants.ARM_PID_P_UP); 
      pidController.setI(Constants.ARM_PID_I);
    }
    else {
      pidController.setP(Constants.ARM_PID_P_DOWN);
      pidController.setI(0);
    }

    pidController.setReference(Math.toRadians(angle), ControlType.kPosition);
  }

  public double getEncoderValue() {
    return encoderValue;
  }

  public double getAngle() {
    return angle;
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }

  public boolean getGoingUp() {
    return goingUp;
  }

  public void setGoingUp(boolean bool) {
    goingUp = bool;
  }
}
