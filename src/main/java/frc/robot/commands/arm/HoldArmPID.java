package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.LoggedCommand;

import java.util.function.DoubleSupplier;

public class HoldArmPID extends LoggedCommand {
    
    private DoubleSupplier substationOffset = () -> 0;
    private Arm arm;
    private double targetRaw;
    private final double scaledValue;

    public HoldArmPID(Arm arm, double scaledValue) {
        this.arm = arm;
        this.scaledValue = scaledValue;
        addRequirements(this.arm);
    }

    public HoldArmPID(Arm arm, double angle, DoubleSupplier substationOffset) {
        this(arm, angle);
        this.substationOffset = substationOffset;
    }

    @Override
    public void initialize() {
        super.initialize();
        targetRaw = (scaledValue / Constants.ARM_ENCODER_CONVERSION_FACTOR) + arm.getArmMinEncoderPosition();
        double originValue = targetRaw;
        targetRaw = originValue + substationOffset.getAsDouble();
        arm.setPID(Constants.ARM_PID_P_IN, Constants.ARM_PID_I_IN, Constants.ARM_PID_D_IN, Constants.ARM_PID_FF_IN);
        Logger.logDouble("/arm/pid/targetValue", targetRaw, Constants.ENABLE_LOGGING);
        if (Constants.ARM_DEBUG) {
            SmartShuffleboard.put("Arm", "HoldArmPID Target Angle", targetRaw);
        }
    }

    @Override
    public void execute() {
        arm.setPidding(true);
        arm.setPIDReference(targetRaw);

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
