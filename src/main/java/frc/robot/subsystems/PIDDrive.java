package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class PIDDrive extends SubsystemBase {
  private double angle;
  private CANSparkMax m_motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private double encoderValue;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private int deviceID;
  
  public PIDDrive() {
    angle = 0;
    deviceID = 45;
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    m_motor.restoreFactoryDefaults();

    pidController = m_motor.getPIDController();
    encoder = m_motor.getEncoder();
    encoderValue = encoder.getPosition();

    kP = 1; 
    kI = 0;
    kD = 0;


    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    
    SmartShuffleboard.put("PID", "P Gain", kP);
    SmartShuffleboard.put("PID", "I Gain", kI);
    SmartShuffleboard.put("PID", "D Gain", kD);
  }

  @Override
  public void periodic() {
    double p = SmartShuffleboard.getDouble("PID", "P Gain", 1);
    double i = SmartShuffleboard.getDouble("PID", "I Gain", 0);
    double d = SmartShuffleboard.getDouble("PID", "D Gain", 0);

    if((p != kP)) { 
      pidController.setP(p); 
      kP = p; 
    }
    if((i != kI)) { 
      pidController.setI(i); 
      kI = i; 
    }
    if((d != kD)) { 
      pidController.setD(d); 
      kD = d; 
    }    pidController.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setVoltage(double voltage) {
    m_motor.set(voltage);
  }

  public double getAngle() {
    return angle;
  }
  public void setAngle(double angle) {
    this.angle = angle;
  }
}
