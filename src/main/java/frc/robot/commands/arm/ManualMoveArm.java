package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    }

    @Override
    public void execute() {
        //if the arm is low and the extender is out
        if ((arm.getEncoderValue() <= Constants.NO_EXTENSION_ZONE) && !(arm.getExtender().safeToLowerArm())) {
            arm.setVoltage(0.0);
        } else {
            arm.setVoltage(power);
        }

    }

    @Override
    public void end(boolean Interrupted) {
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
