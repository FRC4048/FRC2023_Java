package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ManualMoveArm extends LoggedCommand {
    
    private Arm arm;
    private double power;

    public ManualMoveArm(Arm arm, double power) {
        this.arm = arm;
        this.power = power;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        //arm.setPID(Constants.ARM_PID_P_IN, Constants.ARM_PID_I_IN, Constants.ARM_PID_D_IN, Constants.ARM_PID_FF_IN);
    }

    @Override
    public void execute() {
        double currentPid = arm.getPidReference();
        //arm.setPIDReference(currentPid + power);
        arm.setVoltage(power);

    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setVoltage(0.0);
        new HoldArmPID(arm, arm.getAnalogValue()+2).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
