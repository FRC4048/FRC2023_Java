package frc.robot.commands.arm;

import frc.robot.subsystems.ArmPID;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ChangeArmPositionPID extends LoggedCommand {
    private ArmPID arm;
    private double increment;

    public ChangeArmPositionPID(ArmPID arm, double increment) {
        this.arm = arm;
        this.increment = increment;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        //arm.setPID(Constants.ARM_PID_P_IN, Constants.ARM_PID_I_IN, Constants.ARM_PID_D_IN, Constants.ARM_PID_FF_IN);
    }

    @Override
    public void execute() {
        arm.changeArmPos(increment);
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
