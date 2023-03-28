
package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class NewHoldArmPID extends LoggedCommand {

    private final double minVoltage = 0.5;
    private final double maxVoltage = 2.5;
    private final double minAngle = 14;
    private final double maxAngle = 39;
    private final double ratio = (maxAngle-minAngle)/(maxVoltage-minVoltage);
    private Arm arm;
    private double angle;
    private PIDController armPidController;

    public NewHoldArmPID(Arm arm, double angle) {
        this.angle = angle;
        this.arm = arm;
        addRequirements(this.arm);

        armPidController = new PIDController(
          0,
          0,
          0);
    }

    @Override
    public void initialize() {
        super.initialize();
        Logger.logDouble("/arm/pid/angle", angle, Constants.ENABLE_LOGGING);

    }

    @Override
    public void execute() {
        double PIDvoltage = armPidController.calculate(arm.getAnalogValue(), angle);
        double angleVoltage = angle - minAngle;
        angleVoltage = (angleVoltage/ratio) + minVoltage;
        arm.setVoltage(PIDvoltage + angleVoltage);
        SmartShuffleboard.put("Sahil", "Voltage Set", PIDvoltage + angleVoltage);
    }

    @Override
    public void end(boolean Interrupted) {
        super.end(Interrupted);
        arm.setVoltage(0);
        Logger.logDouble("/arm/pid/angle", 0, Constants.ENABLE_LOGGING);

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}