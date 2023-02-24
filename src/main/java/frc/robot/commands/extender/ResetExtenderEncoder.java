package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;

public class ResetExtenderEncoder extends CommandBase {

    private Extender extender;

    private boolean limitReached = false;

    private double startTime;
    
    public ResetExtenderEncoder(Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        limitReached = false;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        boolean revLimit = extender.revLimitReached();
        if (revLimit) {
            extender.resetEncoder();
            limitReached = true;
        } else {
            extender.move(-.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        extender.move(0);
    }

    @Override
    public boolean isFinished() {
        return limitReached || Timer.getFPGATimestamp() - startTime > Constants.TIMEOUT;
    }
}
