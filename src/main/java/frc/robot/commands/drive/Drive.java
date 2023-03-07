package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class Drive extends CommandBase{

    private Drivetrain drivetrain;

    private DoubleSupplier fwdSupplier, strSupplier, rtSupplier;
    private double gyroPos;
    private double turnAtSpeed;
    private JoystickButton decreaseSpeedButton;
    private int closest;


    public Drive(
            Drivetrain drivetrain,
            DoubleSupplier fwdSupplier,
            DoubleSupplier strSupplier,
            DoubleSupplier rtSupplier, JoystickButton decreaseSpeedButton) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.fwdSupplier = fwdSupplier;
        this.strSupplier = strSupplier;
        this.rtSupplier = rtSupplier;

        drivetrain.setTurnToDegreeState(false);
        this.decreaseSpeedButton = decreaseSpeedButton;
    }


    @Override
    public void execute() {
        double mod = decreaseSpeedButton.getAsBoolean() ? Constants.PRECISION_DRIVE_AND_STEER_SPD : 1;
<<<<<<< HEAD
        double fwd = MathUtil.applyDeadband(fwdSupplier.getAsDouble()*Constants.MAX_VELOCITY,0.1);
        double str = MathUtil.applyDeadband(strSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        double rcw = MathUtil.applyDeadband(rtSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        turnAtSpeed = 0;
        closest = -1;
        
        //fwd = fwd * fwd * Math.signum(fwd);
        //str = str * str * Math.+signum(str);
        //rcw = rcw * rcw * Math.signum(rcw);
        if (!drivetrain.isTurnToDegreeOn()) {
            gyroPos = drivetrain.getGyro();
        } else {
            if (!(Math.abs(rcw)>0) & !((Math.abs(drivetrain.getGyro()%180) < 10) & (drivetrain.getGyro()) > -10)) {
                if (Math.abs(drivetrain.getGyro()%180) < 90) {
                    closest = 1;
                }
                turnAtSpeed = Constants.AUTO_TURN_SPEED * Math.signum(drivetrain.getGyro()%180)*closest;
            } else {
                drivetrain.setTurnToDegreeState(false);
            }
        }
=======
        double fwd = MathUtil.applyDeadband(fwdSupplier.getAsDouble()*Constants.MAX_VELOCITY,0.05);
        double str = MathUtil.applyDeadband(strSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.05);
        double rcw = MathUtil.applyDeadband(rtSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.05);
>>>>>>> e5ac88e7458a6a0207b7576a7b2290239493782b

        // TODO: Call drivetrain
        drivetrain.drive(-fwd*mod, -str*mod, -rcw*mod-turnAtSpeed, true);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
