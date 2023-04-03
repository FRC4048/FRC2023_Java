package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.logging.wrappers.LoggedCommand;

import java.util.function.DoubleSupplier;

public class HoldArmPID extends LoggedCommand {

    private final double originAngle;
    private DoubleSupplier substationOffset = () -> 0;
    private Arm arm;
    private double angle;

    public HoldArmPID(Arm arm, double angle) {
        this.angle = angle;
        this.arm = arm;
        this.originAngle = angle;
        addRequirements(this.arm);
    }

    public HoldArmPID(Arm arm, double angle, DoubleSupplier substationOffset) {
        this.arm = arm;
        this.angle = angle;
        this.originAngle = angle;
        this.substationOffset = substationOffset;
        addRequirements(this.arm);
        
    }

    @Override
    public void initialize() {
        super.initialize();
        angle = originAngle + substationOffset.getAsDouble();
        arm.setPID(Constants.ARM_PID_P_IN, Constants.ARM_PID_I_IN, Constants.ARM_PID_D_IN, Constants.ARM_PID_FF_IN);
        Logger.logDouble("/arm/pid/angle", angle, Constants.ENABLE_LOGGING);

    }

    @Override
    public void execute() {
        arm.setPidding(true);
        arm.setPIDReference(angle);
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setPidding(false);
        Logger.logDouble("/arm/pid/angle", 0, Constants.ENABLE_LOGGING);

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
