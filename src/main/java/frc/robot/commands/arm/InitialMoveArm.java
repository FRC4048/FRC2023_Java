package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.logging.wrappers.LoggedCommand;

import java.time.Duration;
import java.util.function.DoubleSupplier;

public class InitialMoveArm extends LoggedCommand {

    private final double orginDestValue;
    private  DoubleSupplier doubleSupplier = ()->0;
    private Arm arm;
    private double desiredScaledValue;
    private double startTime;
    private PIDController armPIDController;
    private SlewRateLimiter accelFilter;
    private Direction direction;

    private enum Direction {
        UP(1.5, 1.0, 0.4, Constants.ARM_MAX_POWER_UP),
        DOWN(0.5, -1.0, 0.2, Constants.ARM_MAX_POWER_DOWN);

        Direction(double boost, double sign, double kP, double maxPower) {
            this.boost = boost;
            this.sign = sign;
            this.kP = kP;
            this.maxPower = maxPower;
        }

        public double boost;
        public double sign;
        public double kP;
        public double maxPower;
    }

    public InitialMoveArm(Arm arm, double desiredScaledValue) {
        this.arm = arm;
        this.desiredScaledValue = desiredScaledValue;
        this.orginDestValue = desiredScaledValue;
        addRequirements(this.arm);
    }
    public InitialMoveArm(Arm arm, double desiredScaledValue, DoubleSupplier doubleSupplier) {
        this.arm = arm;
        this.desiredScaledValue = desiredScaledValue;
        this.orginDestValue = desiredScaledValue;
        this.doubleSupplier = doubleSupplier;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
        accelFilter = new SlewRateLimiter(Constants.ARM_MAX_VOLTAGE_ACCELERATION);
        desiredScaledValue = orginDestValue + doubleSupplier.getAsDouble();
        if (desiredScaledValue > arm.getScaledAnalogEncoderVal()) {
            this.direction = Direction.UP;
            desiredScaledValue += Constants.ARM_OVERSHOOT;
        } else {
            this.direction = Direction.DOWN;
        }
        armPIDController = new PIDController(direction.kP, 0, 0);
    }

    @Override
    public void execute() {
        //positive angle -> positive power
        double currentAngle = arm.getScaledAnalogEncoderVal();
        double armOutput = Math.abs(armPIDController.calculate(currentAngle, desiredScaledValue));
        double filteredOutput = accelFilter.calculate(armOutput);
        double voltage = MathUtil.clamp(filteredOutput, direction.boost, direction.maxPower);
        arm.setVoltage(voltage * direction.sign);
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setVoltage(0.0);
        Logger.logDouble("arm/InitialMoveArmFinishValue",arm.getScaledAnalogEncoderVal(),Constants.ENABLE_LOGGING);
    }

    @Override
    public boolean isFinished() {
        if ((direction == Direction.UP) && ((desiredScaledValue - arm.getScaledAnalogEncoderVal()) <= Constants.ARM_MOVE_PID_THRESHOLD)) {
            return true;
        }
        if ((direction == Direction.DOWN) && ((desiredScaledValue - arm.getScaledAnalogEncoderVal()) >= Constants.ARM_MOVE_PID_THRESHOLD)) {
            return true;
        }
        if ((Timer.getFPGATimestamp() - startTime) > Constants.ARMVOLTAGE_TIMEOUT) {
            Logger.logTimeout(getName(), Constants.ENABLE_LOGGING);
            return true;
        } else {
            return false;
        }
    }
}
