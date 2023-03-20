package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase{

    private Drivetrain drivetrain;

    private DoubleSupplier fwdSupplier, strSupplier, rtSupplier;
    private JoystickButton decreaseSpeedButton;
    private JoystickButton increaseSpeedButton;


    public Drive(
            Drivetrain drivetrain,
            DoubleSupplier fwdSupplier,
            DoubleSupplier strSupplier,
            DoubleSupplier rtSupplier, 
            JoystickButton decreaseSpeedButton,
            JoystickButton increaseSpeedButton) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.fwdSupplier = fwdSupplier;
        this.strSupplier = strSupplier;
        this.rtSupplier = rtSupplier;
        this.decreaseSpeedButton = decreaseSpeedButton;
        this.increaseSpeedButton = increaseSpeedButton;
    }


    @Override
    public void execute() {
        double mod = 1;
        if (decreaseSpeedButton.getAsBoolean()) {
            mod = Constants.PRECISION_DRIVE_AND_STEER_SPD;
        } else if (increaseSpeedButton.getAsBoolean()) {
            mod = Constants.TURBO_DRIVE_AND_STEER_SPD;
        }
        double fwd = MathUtil.applyDeadband(fwdSupplier.getAsDouble()*Constants.MAX_VELOCITY,0.1);
        double str = MathUtil.applyDeadband(strSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        double rcw = MathUtil.applyDeadband(rtSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);

        // TODO: Call drivetrain
        drivetrain.drive(-fwd*mod, -str*mod, -rcw*mod, true);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
