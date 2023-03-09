package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;

public class ResetExtenderEncoder extends CommandBase {

    private final Extender extender;
    private double startTime;
    
    public ResetExtenderEncoder(Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        startTime  = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        extender.move(-Constants.EXTENDER_AUTO_MIN_SPEED);
    }
        

    @Override
    public void end(boolean interrupted) {
        extender.stop();
        extender.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return extender.revLimitReached() || Timer.getFPGATimestamp() - startTime > Constants.EXTENDER_RESET_TIMEOUT;
    }
}

