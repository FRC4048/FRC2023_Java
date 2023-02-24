package frc.robot.commands.extender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;

public class ResetExtenderEncoder extends CommandBase {

    private Extender extender;

    private boolean limitReached = false;
    
    public ResetExtenderEncoder(Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        limitReached = false;
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
        return limitReached;
    }
    
}
