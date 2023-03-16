package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxEncoder;
import frc.robot.utils.diag.DiagSparkMaxSwitch;

public class Arm extends SubsystemBase {
  private double angle;
  private CANSparkMax neoMotor;
  private RelativeEncoder encoder;
  private AnalogInput analogInput;
  public double kP, kI, kD, kIz, kFF, kVoltage;
  private boolean pidding;
  private ProtectionMechanism protectionMechanism;
  private double pidreference;


private SparkMaxPIDController pidController;
  
  public Arm() {

    angle = 0;

    neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
    encoder = neoMotor.getEncoder();
    analogInput = new AnalogInput(Constants.ARM_POT_ID);
    neoMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    neoMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    pidController = neoMotor.getPIDController();

    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Arm", "Pot", Constants.ARM_POT_ID, neoMotor));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm", "Extended Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm", "Retracted Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.REVERSE));


    neoMotor.restoreFactoryDefaults();
    neoMotor.setIdleMode(IdleMode.kBrake);
    // encoder.setPosition(0);

  }

  @Override
  public void periodic() {
    if (Constants.ARM_DEBUG) {
      SmartShuffleboard.put("Arm","arm pot", analogInput.getValue());
      SmartShuffleboard.put("Arm", "arm pidding", pidding);
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
    neoMotor.setVoltage(validateArmVolt(val));
  }

  public void zeroPID() {
    neoMotor.getPIDController().setP(0);
    neoMotor.getPIDController().setI(0);
    neoMotor.getPIDController().setD(0);
    neoMotor.getPIDController().setFF(0);
  }

  public void setPIDReference(double reference) {
    if ((reference > Constants.NO_EXTENSION_ZONE) || (protectionMechanism.safeToLowerArm())) {
      pidreference = reference;
      pidController.setReference(reference, ControlType.kPosition, 0);
  }
  }

  public double getPidReference() {
    return pidreference;
  }

  public CANSparkMax getNeoMotor() {
    return neoMotor;
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public AnalogInput getAnalogInput(){
    return analogInput;
  }

  public void setPID(double p, double i, double d, double f) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setFF(f);
  }

  public void setProtectionMechanism(ProtectionMechanism protectionMechanism) {
    this.protectionMechanism = protectionMechanism;
  }
  public double clampVolts(double value, double min, double max){
    return Math.min(Math.max(value, min), max);
  }
  public double validateArmVolt(double volt){
    volt = clampVolts(volt,-Constants.ARM_MAX_VOLTS,Constants.ARM_MAX_VOLTS);
    if ((volt < 0 && protectionMechanism.safeToLowerArm()) || volt > 0) return volt;
    return 0;
  }
}
