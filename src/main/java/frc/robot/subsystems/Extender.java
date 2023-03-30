package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagTalonSrxEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

public class Extender extends SubsystemBase {
    private WPI_TalonSRX extenderMotor;
    private ProtectionMechanism protectionMechanism;

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

        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Extender", "Encoder", Constants.DIAG_TALONSRX_ROT, extenderMotor));
        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Extender", "Extended Switch", extenderMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Extender", "Retracted Switch", extenderMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
    }

    public void resetEncoder() {
        extenderMotor.setSelectedSensorPosition(0);
    }
    

    public void move(double speed) {
        extenderMotor.set(validateExtenderVolt(speed));
    }

    public void stop() {
        extenderMotor.set(0);
    }

    public double getEncoder() {
        return extenderMotor.getSelectedSensorPosition();
    }

    public boolean fwdLimitReached() {
        return extenderMotor.isFwdLimitSwitchClosed() == 1;
    }

    public boolean revLimitReached() {
        return extenderMotor.isRevLimitSwitchClosed() == 1;
    }
    
    

    @Override
    public void periodic() {
        if (Constants.EXTENDER_DEBUG) {
            SmartShuffleboard.put("Extender", "encoder", getEncoder());
            SmartShuffleboard.put("Extender", "Fwd Limt", fwdLimitReached());
            SmartShuffleboard.put("Extender", "Rev Limit", revLimitReached());
        }
        Logger.logDouble("/extender/encoder",getEncoder(), Constants.ENABLE_LOGGING);
        Logger.logBoolean("/extender/fwdLimit", fwdLimitReached(),Constants.ENABLE_LOGGING);
        Logger.logBoolean("/extender/revLimit", revLimitReached(),Constants.ENABLE_LOGGING);
    }
    

    public void setProtectionMechanism(ProtectionMechanism protectionMechanism) {
        this.protectionMechanism = protectionMechanism;
    }
    public double validateExtenderVolt(double volt){
        if ((volt > 0 && protectionMechanism.safeToExtend()) || volt < 0) return volt;
        return 0;
    }
}
