package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public class NeoSubsystem extends SubsystemBase {
    private String name;
    private CANSparkMax sparkMax;
    RelativeEncoder encoder;
    SparkMaxLimitSwitch forwardLimitSwitch;
    SparkMaxLimitSwitch reverseLimitSwitch;

    public NeoSubsystem(String name, int canId) {
        this.name = name;
        sparkMax = new CANSparkMax(canId, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = sparkMax.getEncoder();
        forwardLimitSwitch = sparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        reverseLimitSwitch = sparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        sparkMax.restoreFactoryDefaults();
    }

    public void drive(double speed) {
        sparkMax.set(speed);
    }

    public void stop() {
        sparkMax.set(0.0);
    }

    public double getEncoder() {
        return encoder.getPosition();
    }

    public boolean getForwardSwitch() {
        return forwardLimitSwitch.isPressed();
    }

    public boolean getReverseSwitch() {
        return reverseLimitSwitch.isPressed();
    }

    @Override
    public void periodic() {
        SmartShuffleboard.put(Constants.TEST_MOTOR, name + " encoder", getEncoder());
        SmartShuffleboard.put(Constants.TEST_MOTOR, name + " forward switch", getForwardSwitch());
        SmartShuffleboard.put(Constants.TEST_MOTOR, name + " reverse switch", getReverseSwitch());
    }
}
