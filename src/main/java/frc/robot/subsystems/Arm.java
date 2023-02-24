package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmMoveSequence;
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
  private Constants.Grid SelectedGridSlot;

  
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
    displayGrid();
    if (Constants.DEBUG) {
      SmartShuffleboard.put("Arm", "arm encoder", (getEncoderValue()));
      SmartShuffleboard.put("Arm", "arm pidding", pidding);
    }
  }

  public void displayGrid() {
    SmartShuffleboard.put("test", "1", "upLeft", isSlotSelected(Constants.Grid.UP_LEFT));
    SmartShuffleboard.put("test", "1", "middleLeft", isSlotSelected(Constants.Grid.MIDDLE_LEFT));
    SmartShuffleboard.put("test", "1", "downLeft", isSlotSelected(Constants.Grid.DOWN_LEFT));

    SmartShuffleboard.put("test", "2", "upMiddle", isSlotSelected(Constants.Grid.UP_MIDDLE));
    SmartShuffleboard.put("test", "2", "middleMiddle", isSlotSelected(Constants.Grid.MIDDLE_MIDDLE));
    SmartShuffleboard.put("test", "2", "downMiddle", isSlotSelected(Constants.Grid.DOWN_MIDDLE));

    SmartShuffleboard.put("test", "3", "upRight", isSlotSelected(Constants.Grid.UP_RIGHT));
    SmartShuffleboard.put("test", "3", "middleRight", isSlotSelected(Constants.Grid.MIDDLE_RIGHT));
    SmartShuffleboard.put("test", "3", "downRight", isSlotSelected(Constants.Grid.DOWN_LEFT));
  }

  public boolean isSlotSelected(Constants.Grid slot) {
    return slot == SelectedGridSlot;
  }

  public Constants.Grid getSelectedGridSlot() {
    return SelectedGridSlot;
  }

  public void setSelectedGridSlot(Constants.Grid slot) {
    SelectedGridSlot = slot;
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
    if (val > Constants.ARM_MAX_VOLTS) {
    neoMotor.setVoltage(Constants.ARM_MAX_VOLTS);
  } else if (val < -Constants.ARM_MAX_VOLTS) {
    neoMotor.setVoltage(-Constants.ARM_MAX_VOLTS);}
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
