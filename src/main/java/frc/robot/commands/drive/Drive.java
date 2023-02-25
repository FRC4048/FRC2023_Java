package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase{

    private Drivetrain drivetrain;

    private DoubleSupplier fwdSupplier, strSupplier, rtSupplier;

    
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
    }


    @Override
    public void execute() {
//Square the values for finer movement
        double fwd = MathUtil.applyDeadband(fwdSupplier.getAsDouble()*Constants.MAX_VELOCITY,0.1);
        double str = MathUtil.applyDeadband(strSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        double rcw = MathUtil.applyDeadband(rtSupplier.getAsDouble()*Constants.MAX_VELOCITY, 0.1);
        
        //fwd = fwd * fwd * Math.signum(fwd);
        //str = str * str * Math.signum(str);
        //rcw = rcw * rcw * Math.signum(rcw);

        // TODO: Call drivetrain
        drivetrain.drive(-fwd, -str, -rcw, true);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
