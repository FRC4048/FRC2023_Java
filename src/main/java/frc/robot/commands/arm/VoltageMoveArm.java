package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class VoltageMoveArm extends CommandBase {
    
    private Arm arm;
    private Double power;
    private Double angle;

    public VoltageMoveArm(Arm arm, Double power, Double angle) {
        this.arm = arm;
        this.power = power;
        this.angle = angle;
        addRequirements(this.arm);

        if (angle > arm.getEncoderValue()) {
        arm.setVoltage(power);
        } else {
        arm.setVoltage(-power);
        }
    }

    @Override
    public void execute() {
        //negative value moves arm up
    }

    @Override
    public void end(boolean Interrupted) {
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return angle - arm.getEncoderValue() < 4;
    }

    
}
