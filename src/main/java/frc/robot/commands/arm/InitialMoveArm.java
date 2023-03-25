package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class InitialMoveArm extends LoggedCommand {

    private Arm arm;
    private double desiredAngle;
    private double startTime;
    private PIDController armPIDController;
    private SlewRateLimiter accelFilter;
    private Direction direction;

    private enum Direction {
        UP(1.5, 1.0, 0.6, Constants.ARM_MAX_POWER_UP),
        DOWN(0.5, -1.0, 0.3, Constants.ARM_MAX_POWER_DOWN);

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

    public InitialMoveArm(Arm arm, double desiredAngle) {
        this.arm = arm;
        this.desiredAngle = desiredAngle;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
        accelFilter = new SlewRateLimiter(Constants.ARM_MAX_VOLTAGE_ACCELERATION);
        if (desiredAngle > arm.getAnalogValue()) {
            this.direction = Direction.UP;
            desiredAngle += Constants.ARM_OVERSHOOT;
        } else {
            this.direction = Direction.DOWN;
        }
        armPIDController = new PIDController(direction.kP, 0, 0);
    }

    @Override
    public void execute() {
        //positive angle -> positive power
        double currentAngle = arm.getAnalogValue();
        double armOutput = Math.abs(armPIDController.calculate(currentAngle,desiredAngle));
        double filteredOutput = accelFilter.calculate(armOutput);
        double voltage = MathUtil.clamp(filteredOutput, direction.boost, direction.maxPower);

        SmartShuffleboard.put("bat", "error", desiredAngle - currentAngle);
        SmartShuffleboard.put("bat", "filtered output", filteredOutput);
        SmartShuffleboard.put("bat", "arm output", armOutput);
        SmartShuffleboard.put("bat", "voltage", voltage);
        arm.setVoltage(voltage * direction.sign);
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(desiredAngle - arm.getAnalogValue()) < Constants.ARM_MOVE_PID_THRESHOLD) || ((Timer.getFPGATimestamp() - startTime) > Constants.ARMVOLTAGE_TIMEOUT);
    }
}
