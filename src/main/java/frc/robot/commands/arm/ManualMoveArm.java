package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class ManualMoveArm extends CommandBase {
    
    private Arm arm;
    private double power;

    public ManualMoveArm(Arm arm, double power) {
        this.arm = arm;
        this.power = power;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
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
        arm.setVoltage(0.0);
        new HoldArmPID(arm, arm.getAnalogValue()).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
