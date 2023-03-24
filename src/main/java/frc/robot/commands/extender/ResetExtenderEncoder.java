package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ResetExtenderEncoder extends LoggedCommand {

    private final Extender extender;
    private double startTime;
    
    public ResetExtenderEncoder(Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime  = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        extender.move(-Constants.EXTENDER_AUTO_MIN_SPEED);
        }
        

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        extender.stop();
        extender.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return extender.revLimitReached() || Timer.getFPGATimestamp() - startTime > Constants.EXTENDER_RESET_TIMEOUT;
    }
}

