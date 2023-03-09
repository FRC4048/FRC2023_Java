package frc.robot.commands.extender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;

public class ResetExtenderEncoder extends CommandBase {

    private final Extender extender;
    
    public ResetExtenderEncoder(Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize() {}

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
        return extender.revLimitReached();
    }
}

