package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        //negative value moves arm up
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
        return Math.abs(angle - arm.getEncoderValue()) < 2.5;
    }

    
}
