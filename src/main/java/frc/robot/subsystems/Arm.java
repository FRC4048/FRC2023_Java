package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxEncoder;
import frc.robot.utils.diag.DiagSparkMaxSwitch;
import frc.robot.utils.diag.DiagToFSensor;

/**
 * The RevRobotics distance sensor requires manual installation to work correctly.
 * 
 * Follow the instruments on this page to install the driver:
 * 
 * https://github.com/REVrobotics/2m-Distance-Sensor
 */
public class Arm extends SubsystemBase {
  private CANSparkMax neoMotor;
  private RelativeEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kVoltage;
  private boolean pidding;
  private ProtectionMechanism protectionMechanism;
  private Rev2mDistanceSensor distanceSensor;
  private double pidreference;
  private SparkMaxAnalogSensor analogSensor;
  private SparkMaxPIDController pidController;
  
  public Arm() {
    neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
    encoder = neoMotor.getEncoder();
    analogSensor = neoMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    neoMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    neoMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    pidController = neoMotor.getPIDController();

    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Arm", "Encoder", Constants.DIAG_SPARK_ROT, neoMotor));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm", "Extended Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxSwitch("Arm", "Retracted Switch", neoMotor, frc.robot.utils.diag.DiagSparkMaxSwitch.Direction.REVERSE));
    Robot.getDiagnostics().addDiagnosable(new DiagToFSensor("Arm", "Time of Flight", distanceSensor, 5, 15));

    neoMotor.restoreFactoryDefaults();
    neoMotor.setIdleMode(IdleMode.kBrake);
    encoder.setPosition(0);

    distanceSensor.setAutomaticMode(true);
    distanceSensor.setDistanceUnits(Unit.kInches);
    distanceSensor.setRangeProfile(RangeProfile.kHighSpeed);


  }

  @Override
  public void periodic() {
    if (Constants.ARM_DEBUG) {

      SmartShuffleboard.put("Arm", "arm encoder", getEncoderValue());
      SmartShuffleboard.put("Arm", "analog encoder",getAnalogValue());
      SmartShuffleboard.put("Arm", "analog raw encoder",analogSensor.getPosition());
      SmartShuffleboard.put("Arm", "arm pidding", pidding);
      SmartShuffleboard.put("Arm", "P Gain", pidController.getP());
      SmartShuffleboard.put("Arm", "I Gain", pidController.getI());
      SmartShuffleboard.put("Arm", "D Gain", pidController.getD());
      SmartShuffleboard.put("Arm", "FF Gain", pidController.getFF());
      SmartShuffleboard.put("Arm", "Distance Sensor Inches", distanceSensor.getRange(Unit.kInches));
      }
    Logger.logDouble("/arm/analogEncoder", getAnalogValue(), Constants.ENABLE_LOGGING);
    Logger.logBoolean("/arm/fwdLimit", isFwdLimitSwitchReached(),Constants.ENABLE_LOGGING);
    Logger.logBoolean("/arm/revLimit",isRevLimitSwitchReached(),Constants.ENABLE_LOGGING);
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

  public double getAnalogValue(){
    return (analogSensor.getPosition() - Constants.ARM_MIN_ENC_VAL) * Constants.ARM_ENCODER_CONVERSION_FACTOR;
  }

  public void setPidding(boolean bool) {
    pidding = bool;
  }

  public void setVoltage(double val) {
    neoMotor.setVoltage(validateArmVolt(val));
  }

  public SparkMaxAnalogSensor getAnalogSensor(){
    return analogSensor;
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

  public double getDistance() {
    return distanceSensor.getRange();
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
