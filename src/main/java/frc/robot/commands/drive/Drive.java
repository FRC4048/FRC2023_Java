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
<<<<<<< HEAD
    private double gyroPos;
    private double turnAtSpeed;
=======
    private JoystickButton decreaseSpeedButton;

>>>>>>> ad4f9a61ea314d33a22bfb516a7bb6332559d8c5

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
<<<<<<< HEAD

        drivetrain.setTurnToDegreeState(false);
=======
        this.decreaseSpeedButton = decreaseSpeedButton;
>>>>>>> ad4f9a61ea314d33a22bfb516a7bb6332559d8c5
    }


    @Override
    public void execute() {
        double mod = decreaseSpeedButton.getAsBoolean() ? Constants.PRECISION_DRIVE_AND_STEER_SPD : 1;
        double fwd = MathUtil.applyDeadband(fwdSupplier.getAsDouble()*Constants.MAX_VELOCITY,0.1);
        double str = MathUtil.applyDeadband(strSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        double rcw = MathUtil.applyDeadband(rtSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
<<<<<<< HEAD
        turnAtSpeed = 0;
        
        //fwd = fwd * fwd * Math.signum(fwd);
        //str = str * str * Math.+signum(str);
        //rcw = rcw * rcw * Math.signum(rcw);
        if (!drivetrain.isTurnToDegreeOn()) {
            gyroPos = drivetrain.getGyro();
        } else {
            if (!(Math.abs(rcw)>0) & !((Math.abs(drivetrain.getGyro()) < drivetrain.getDegreesToTurn() + 3) & (drivetrain.getGyro()) > drivetrain.getDegreesToTurn() - 3)) {
                turnAtSpeed = Constants.AUTO_TURN_SPEED * Math.signum(drivetrain.getGyro()-180);
            } else {
                drivetrain.setTurnToDegreeState(false);
            }
        }
        // TODO: Call drivetrain
        drivetrain.drive(-fwd, -str, -rcw-turnAtSpeed, true);
=======

        // TODO: Call drivetrain
        drivetrain.drive(-fwd*mod, -str*mod, -rcw*mod, true);
>>>>>>> ad4f9a61ea314d33a22bfb516a7bb6332559d8c5
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
