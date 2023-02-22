package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

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
  private RelativeEncoder encoder;
  private double encoderValue;
  public double kP, kI, kD, kIz, kFF, kVoltage;
  private boolean pidding;

  
  public Arm() {
    angle = 0;

    neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
    encoder = neoMotor.getEncoder();

    neoMotor.restoreFactoryDefaults();
    neoMotor.setIdleMode(IdleMode.kBrake);
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    encoderValue = encoder.getPosition();
    SmartShuffleboard.put("PID", "pidding", pidding);
    
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

  public void setPidding(boolean bool) {
    pidding = bool;
  }

  public void setVoltage(Double val) {
    if (val > 4.5) {
    neoMotor.setVoltage(4.5);
  } else if (val < -4.5) {
    neoMotor.setVoltage(-4.5);}
    else {
      neoMotor.setVoltage(val);
    }
  }

  public CANSparkMax getNeoMotor() {
    return neoMotor;
  }
}
