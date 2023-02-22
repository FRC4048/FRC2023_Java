package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase{

    private Drivetrain drivetrain;

    private DoubleSupplier fwdSupplier, strSupplier, rtSupplier;
    private double gyroPos;
    private double turnAtSpeed;

    
    public Drive(
        Drivetrain drivetrain, 
        DoubleSupplier fwdSupplier, 
        DoubleSupplier strSupplier, 
        DoubleSupplier rtSupplier) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.fwdSupplier = fwdSupplier;
        this.strSupplier = strSupplier;
        this.rtSupplier = rtSupplier;

        drivetrain.setTurnToDegreeState(false);
    }


    @Override
    public void execute() {
//Square the values for finer movement
        double fwd = MathUtil.applyDeadband(fwdSupplier.getAsDouble()*Constants.MAX_VELOCITY,0.1);
        double str = MathUtil.applyDeadband(strSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        double rcw = MathUtil.applyDeadband(rtSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
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
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
