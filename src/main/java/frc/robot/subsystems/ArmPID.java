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

    private double pidP = 5e-5;
    private double pidI = 1e-6;
    private double pidD = 0;
    private double armpos = 0.0;
    private double iZone = 0.0;
    private double ff = 0.000156;

    public ArmPID() {
        neoMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
        neoMotor.restoreFactoryDefaults();
        encoder = neoMotor.getEncoder();
        neoMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        neoMotor.getReverseLimitSwitch(Type.kNormallyOpen);

        pidController = neoMotor.getPIDController();
        pidController.setP(pidP);
        pidController.setI(pidI);
        pidController.setD(pidD);
        pidController.setIZone(iZone);
        pidController.setFF(ff);
        pidController.setOutputRange(-1, 1);

        pidController.setSmartMotionMaxVelocity(500.0, 0);
        pidController.setSmartMotionMinOutputVelocity(0.0, 0);
        pidController.setSmartMotionMaxAccel(1500.0, 0);
        pidController.setSmartMotionAllowedClosedLoopError(0.0, 0);

        SmartShuffleboard.put("Arm", "PID P", pidP);
        SmartShuffleboard.put("Arm", "PID I", pidI);
        SmartShuffleboard.put("Arm", "PID D", pidD);
    }

    public void periodic() {

        SmartShuffleboard.put("Arm", "P Gain", pidController.getP());
        SmartShuffleboard.put("Arm", "I Gain", pidController.getI());
        SmartShuffleboard.put("Arm", "D Gain", pidController.getD());
        SmartShuffleboard.put("Arm", "FF Gain", pidController.getFF());
        SmartShuffleboard.put("Arm", "Encoder Value", encoder.getPosition());
        SmartShuffleboard.put("Arm", "Desired pos", armpos);
        //pid tuning
        pidP = SmartShuffleboard.getDouble("Arm", "PID P", pidP);
        pidI = SmartShuffleboard.getDouble("Arm", "PID I", pidI);
        pidD = SmartShuffleboard.getDouble("Arm", "PID D", pidD);
    }

    public void setArmPos(double position) {
        pidController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
        this.armpos = position;
    }

    public void changeArmPos(double increment) {
        double newPos = Math.max(0.0, Math.min(40.0, this.armpos + increment));
        setArmPos(newPos);
    }

    public void setPid() {
        pidController.setP(pidP);
        pidController.setI(pidI);
        pidController.setD(pidD);

        System.out.println("-------------->>>>>>> Set pid to " + pidP + "/" + pidI + "/" + pidD);
    }
}
