package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public class TalonSubsystem extends SubsystemBase {
    private String name;
    private TalonSRX talonSRX;

    public TalonSubsystem(String name, int canId) {
        this.name = name;
        talonSRX = new TalonSRX(canId);
    }

    public void drive(double speed) {
        talonSRX.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        talonSRX.set(ControlMode.PercentOutput, 0.0);
    }

    public double getEncoder() {
        return talonSRX.getSensorCollection().getQuadraturePosition();
    }

    public boolean getForwardSwitch() {
        return talonSRX.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseSwitch() {
        return talonSRX.getSensorCollection().isRevLimitSwitchClosed();
    }

    @Override
    public void periodic() {
        SmartShuffleboard.put(Constants.TEST_MOTOR, name + " encoder", getEncoder());
        SmartShuffleboard.put(Constants.TEST_MOTOR, name + " forward switch", getForwardSwitch());
        SmartShuffleboard.put(Constants.TEST_MOTOR, name + " reverse switch", getReverseSwitch());
    }
}
