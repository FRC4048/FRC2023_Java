package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class VoltageMoveArm extends CommandBase {

    private Arm arm;
    private Double upPower;
    private Double downPower;
    private Double angle;

    public VoltageMoveArm(Arm arm, Double upPower, Double downPower, Double angle) {
        this.arm = arm;
        this.upPower = upPower;
        this.downPower = downPower;
        this.angle = angle;
        addRequirements(this.arm);


    }

    @Override
    public void initialize() {

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
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angle - arm.getEncoderValue()) < Constants.ARM_MOVE_PID_THRESHOLD;
    }


}
