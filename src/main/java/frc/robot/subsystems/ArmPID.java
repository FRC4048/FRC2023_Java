package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPID extends SubsystemBase {
    private CANSparkMax neoMotor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pidController;
    private double pidP = Constants.ARM_PID_P_IN;
    private double pidI = Constants.ARM_PID_I_IN;
    private double pidD = Constants.ARM_PID_D_IN;

    public ArmPID() {
        neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
        neoMotor.restoreFactoryDefaults();
        encoder = neoMotor.getEncoder();
        neoMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        neoMotor.getReverseLimitSwitch(Type.kNormallyOpen);

        pidController = neoMotor.getPIDController();
        pidController.setSmartMotionMaxVelocity(500.0, 0);
        pidController.setSmartMotionMinOutputVelocity(0.0, 0);
        pidController.setSmartMotionMaxAccel(1500.0, 0);
        pidController.setSmartMotionAllowedClosedLoopError(0.0, 0);
    }

    public void periodic() {
        
      SmartShuffleboard.put("Arm", "P Gain", pidController.getP());
      SmartShuffleboard.put("Arm", "I Gain", pidController.getI());
      SmartShuffleboard.put("Arm", "D Gain", pidController.getD());
      SmartShuffleboard.put("Arm", "FF Gain", pidController.getFF());
      SmartShuffleboard.put("Arm", "Encoder Value", encoder.getPosition());
      //pid tuning
      SmartShuffleboard.put("Arm","PID P",pidP);
      SmartShuffleboard.put("Arm","PID I",pidI);
      SmartShuffleboard.put("Arm","PID D",pidD);
      pidP = SmartShuffleboard.getDouble("Arm","PID P",pidP);
      pidI = SmartShuffleboard.getDouble("Arm","PID I",pidI);
      pidD = SmartShuffleboard.getDouble("Arm","PID D",pidD);
    }
    public void setArmPos(double position) {
        pidController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }
}
