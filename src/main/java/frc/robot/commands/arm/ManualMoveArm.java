package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class ManualMoveArm extends CommandBase {
    
    private Arm arm;
    private Double power;

    public ManualMoveArm(Arm arm, Double power) {
        this.arm = arm;
        this.power = power;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        //SmartShuffleboard.put("PID", "recent command", "ManualMoveArm");
    }

    @Override
    public void execute() {
        // Flipping sign, "down" is positive
        this.arm.setVoltage(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
