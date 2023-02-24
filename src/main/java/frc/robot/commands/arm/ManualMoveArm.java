package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class ManualMoveArm extends CommandBase {
    
    private Arm arm;
    private Double power;
    private double startTime;

    public ManualMoveArm(Arm arm, Double power) {
        this.arm = arm;
        this.power = power;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // Flipping sign, "down" is positive
        this.arm.setVoltage(power);
    }

    @Override
    public void end(boolean Interrupted) {
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > Constants.MANUAL_TIMEOUT;
    }

    
}
