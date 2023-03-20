package frc.robot.commands.arm;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.Logging;

public class HoldArmPID extends CommandBase {

    private Arm arm;
    private Double angle;

    public HoldArmPID(Arm arm, Double angle) {
        this.angle = angle;
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        arm.setPID(Constants.ARM_PID_P_IN, Constants.ARM_PID_I_IN, Constants.ARM_PID_D_IN, Constants.ARM_PID_FF_IN);
    }

    @Override
    public void execute() {
        arm.setPidding(true);
        arm.setPIDReference(angle);
    }

    @Override
    public void end(boolean Interrupted) {
        arm.setPidding(false);
        Logger.logDouble("arm/pid/voltage", arm.getNeoMotor().getBusVoltage(), Constants.ENABLE_LOGGING);
        Logger.logDouble("arm/pid/current", angle, Constants.ENABLE_LOGGING);
        Logger.logDouble("arm/pid/desired", arm.getAnalogValue(), Constants.ENABLE_LOGGING);

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
