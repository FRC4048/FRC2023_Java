package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class InitialMoveArm extends LoggedCommand {

    private Arm arm;
    private double desiredAngle;
    private double currentAngle;
    private double startAngle;
    private double startTime;
    private ProfiledPIDController armPIDController;
    private double armMinimumSpeed;

    public InitialMoveArm(Arm arm, double currentAngle, double desiredAngle) {
        this.arm = arm;
        this.desiredAngle = desiredAngle;
        this.startAngle = currentAngle;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        currentAngle = startAngle;
        startTime = Timer.getFPGATimestamp();
        if (desiredAngle > currentAngle) {
            // going up
            armMinimumSpeed = 0; //was 1.5
            armPIDController = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(5.0, 2.0));
        } else {
            // going down
            armMinimumSpeed = 0;
            armPIDController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(5.0, 1.0));
        }
    }

    @Override
    public void execute() {
        //positive angle -> positive power
        final double armOutput = armPIDController.calculate(currentAngle, desiredAngle);
        double voltage = -armOutput + armMinimumSpeed;
        SmartShuffleboard.put("bat", "error", desiredAngle - currentAngle);
        SmartShuffleboard.put("bat", "voltage", voltage);
        SmartShuffleboard.put("bat", "pid output", armOutput);
        currentAngle += 0.1;
        arm.setVoltage(voltage);
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        Logger.logDouble("/arm/angle", currentAngle, Constants.ENABLE_LOGGING);
        Logger.logDouble("/arm/analogValue", arm.getAnalogValue(), Constants.ENABLE_LOGGING);
        return (Math.abs(desiredAngle - currentAngle) < Constants.ARM_MOVE_PID_THRESHOLD);
    }
}
