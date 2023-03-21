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
        double fwd = MathUtil.applyDeadband(fwdSupplier.getAsDouble()*Constants.MAX_VELOCITY,0.1);
        double str = MathUtil.applyDeadband(strSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        double rcw = MathUtil.applyDeadband(rtSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        
        if (decreaseSpeedButton.getAsBoolean()) {
            drivetrain.drive(-0.1 * Math.signum(fwd), -0.1 * Math.signum(str), -0.1 * Math.signum(rcw), true);
        } else {
            drivetrain.drive(-fwd, -str, -rcw, true);
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
