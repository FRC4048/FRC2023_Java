package frc.robot.commands.arm;

import frc.robot.subsystems.ArmPID;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class SetArmPID extends LoggedCommand {
    private ArmPID arm;

    public SetArmPID(ArmPID arm) {
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        arm.setPid();
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
