package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class VoltageMoveArm extends LoggedCommand {

    private Arm arm;
    private double upPower;
    private double downPower;
    private double angle;
    private double startTime;

    public VoltageMoveArm(Arm arm, double upPower, double downPower, double angle) {
        this.arm = arm;
        this.upPower = upPower;
        this.downPower = downPower;
        this.angle = angle;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        //positive angle -> positive power
        if (angle > arm.getAnalogValue()) {
            arm.setVoltage(2.5);
        } else {
            arm.setVoltage(2.5);
        }
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        Logger.logDouble("/arm/angle", angle, Constants.ENABLE_LOGGING);
        Logger.logDouble("/arm/analogValue", arm.getAnalogValue(), Constants.ENABLE_LOGGING);


        return (Math.abs(angle - arm.getAnalogValue()) < Constants.ARM_MOVE_PID_THRESHOLD) || ((Timer.getFPGATimestamp() - startTime) > Constants.ARMVOLTAGE_TIMEOUT);
    }


}
