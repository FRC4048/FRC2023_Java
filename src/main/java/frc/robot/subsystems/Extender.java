package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public class Extender extends SubsystemBase {

    private WPI_TalonSRX extenderMotor;

    public Extender() {
        int TIMEOUT = 100;

        extenderMotor = new WPI_TalonSRX(Constants.EXTENDER_MOTOR_ID);
        extenderMotor.configNominalOutputForward(0, TIMEOUT);
        extenderMotor.configNominalOutputReverse(0, TIMEOUT);
        extenderMotor.configPeakOutputForward(1, TIMEOUT);
        extenderMotor.configPeakOutputReverse(-1, TIMEOUT);
        extenderMotor.setNeutralMode(NeutralMode.Brake);
        extenderMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT);
        extenderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        extenderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        extenderMotor.setSelectedSensorPosition(0);
    }

    public void move(double speed) {
        extenderMotor.set(speed);
    }

    public void stop() {
        extenderMotor.set(0);
    }

    public double getEncoder() {
        return extenderMotor.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        SmartShuffleboard.put("Extender", "encoder",getEncoder());
    }
}
