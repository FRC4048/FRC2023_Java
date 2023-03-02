package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class VoltageMoveArm extends CommandBase {

    private Arm arm;
    private Double power;
    private Double angle;

    public VoltageMoveArm(Arm arm, Double power, Double angle) {
        this.arm = arm;
        this.power = power;
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
            arm.setVoltage(power);
        } else {
            arm.setVoltage(-power);
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
