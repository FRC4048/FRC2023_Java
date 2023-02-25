package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class VoltageMoveArm extends CommandBase {

    private Arm arm;
    private Double power;
    private Double angle;
    private static int initCount = 0;

    public VoltageMoveArm(Arm arm, Double power, Double angle) {
        this.arm = arm;
        this.power = power;
        this.angle = angle;
        addRequirements(this.arm);


    }

    @Override
    public void initialize() {
        SmartShuffleboard.put("Arm","armVoltStatus","Starting");
        initCount++;
        SmartShuffleboard.put("Arm","initCount",initCount);
    }

    @Override
    public void execute() {
        //negative value moves arm up
        if (angle > arm.getEncoderValue()) {
            arm.setVoltage(power);
        } else {
            arm.setVoltage(-power);
        }
        SmartShuffleboard.put("Arm","armVoltStatus","moving");
    }

    @Override
    public void end(boolean Interrupted) {
        arm.setVoltage(0.0);
        SmartShuffleboard.put("Arm","armVoltStatus","ending");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angle - arm.getEncoderValue()) < Constants.ARM_MOVE_PID_THRESHOLD;
    }


}
