package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
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
        if (angle > arm.getEncoderValue()) {
            arm.setVoltage(upPower);
        } else {
            arm.setVoltage(-downPower);
        }
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(angle - arm.getEncoderValue()) < Constants.ARM_MOVE_PID_THRESHOLD) || ((Timer.getFPGATimestamp() - startTime) > Constants.ARMVOLTAGE_TIMEOUT);
    }


}
