package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmMoveSequence;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxEncoder;
import frc.robot.utils.diag.DiagSparkMaxSwitch;

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
    encoderValue = encoder.getPosition();
    if (Constants.DEBUG) {
      SmartShuffleboard.put("Gripper", "arm encoder", (getEncoderValue()));
      SmartShuffleboard.put("Arm", "arm pidding", pidding);
    }
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

  public void zeroPID() {
    neoMotor.getPIDController().setP(0);
    neoMotor.getPIDController().setI(0);
    neoMotor.getPIDController().setD(0);
    neoMotor.getPIDController().setFF(0);
  }

  public CANSparkMax getNeoMotor() {
    return neoMotor;
  }
}
