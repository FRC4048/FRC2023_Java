package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
  private RelativeEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kVoltage;
  private boolean pidding;


private Extender extender;
private SparkMaxPIDController pidController;
  
  public Arm(Extender extender) {
    this.extender = extender;

    angle = 0;

    neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
    encoder = neoMotor.getEncoder();
    neoMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    neoMotor.getReverseLimitSwitch(Type.kNormallyOpen);



    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Arm", "Encoder", Constants.DIAG_SPARK_ROT, neoMotor));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm", "Extended Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm", "Retracted Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.REVERSE));


    neoMotor.restoreFactoryDefaults();
    neoMotor.setIdleMode(IdleMode.kBrake);
    encoder.setPosition(0);

  }

  @Override
  public void periodic() {
    if (Constants.DEBUG) {
      SmartShuffleboard.put("Arm", "arm encoder", (getEncoderValue()));
      SmartShuffleboard.put("Arm", "arm pidding", pidding);
    }

    if (Constants.DEBUG) {
      SmartShuffleboard.put("Arm", "P Gain", pidController.getP());
      SmartShuffleboard.put("Arm", "I Gain", pidController.getI());
      SmartShuffleboard.put("Arm", "D Gain", pidController.getD());
      SmartShuffleboard.put("Arm", "FF Gain", pidController.getFF());
      }
  }

  public boolean isFwdLimitSwitchReached() {
    return neoMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public boolean isRevLimitSwitchReached() {
    return neoMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public double getEncoderValue() {
    return encoder.getPosition();
  }

  public boolean safeToExtend() {
    return (encoderValue > Constants.NO_EXTENSION_ZONE);
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
    if (val < 1 && !safeToExtend() && !extender.safeToLowerArm()) {
      setVoltage(0.0);
    } else {
      if (val > Constants.ARM_MAX_VOLTS) {
        neoMotor.setVoltage(Constants.ARM_MAX_VOLTS);
      } else if (val < -Constants.ARM_MAX_VOLTS) {
      neoMotor.setVoltage(-Constants.ARM_MAX_VOLTS);}
      else {
        neoMotor.setVoltage(val);
      }
    }
  }

  public void zeroPID() {
    neoMotor.getPIDController().setP(0);
    neoMotor.getPIDController().setI(0);
    neoMotor.getPIDController().setD(0);
    neoMotor.getPIDController().setFF(0);
  }

  public void setPIDReference(double reference) {
    if ((reference > Constants.NO_EXTENSION_ZONE) || (extender.safeToLowerArm())) {
      pidController.setReference(reference, ControlType.kPosition, 0);
  }
  }

  public CANSparkMax getNeoMotor() {
    return neoMotor;
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public void setPID(double p, double i, double d, double f) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setFF(f);
  }

  public Extender getExtender() {
    return extender;
  }
  
}
