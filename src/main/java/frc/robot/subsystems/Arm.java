package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxEncoder;
import frc.robot.utils.diag.DiagSparkMaxSwitch;

public class Arm extends SubsystemBase {
  private double angle;
  private CANSparkMax neoMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private double encoderValue;
  public double kP, kI, kD, kIz, kFF, kVoltage;
  
  public Arm() {
    angle = 0;

    neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
    encoder = neoMotor.getEncoder();  
    neoMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    neoMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Arm Encoder", Constants.DIAG_SPARK_ROT, neoMotor));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm Open Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm Close Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.REVERSE));


    neoMotor.restoreFactoryDefaults();
    neoMotor.setIdleMode(IdleMode.kBrake);
    encoder.setPosition(0);

    pidController = neoMotor.getPIDController();
    
    kP = Constants.ARM_PID_P; 
    kI = Constants.ARM_PID_I;
    kD = Constants.ARM_PID_D;
    kFF = Constants.ARM_PID_FF;
  
    pidController.setP(kP);
    pidController.setI(kI);
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
    
    double p = SmartShuffleboard.getDouble("PID", "P Gain", 1);
    double i = SmartShuffleboard.getDouble("PID", "I Gain", 0);
    double d = SmartShuffleboard.getDouble("PID", "D Gain", 0);
    double ff = SmartShuffleboard.getDouble("PID", "FF Gain", 0);

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
    }
    if((ff != kFF)) { 
      pidController.setFF(ff); 
      kFF = ff; 
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
}
