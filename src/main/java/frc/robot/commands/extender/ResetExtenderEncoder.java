package frc.robot.commands.extender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;

public class ResetExtenderEncoder extends CommandBase {

    private Extender extender;

    private boolean limitReached = false;

    private boolean ResetEncoders;
    
    public ResetExtenderEncoder(Extender extender, boolean ResetEncoders) {
        this.extender = extender;
        this.ResetEncoders = ResetEncoders;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        limitReached = false;
    }

    @Override
    public void execute() {
        if (ResetEncoders == true) {
            boolean revLimit = extender.revLimitReached();
            if (revLimit) {
                extender.resetEncoder();
                limitReached = true;
            } else {
                extender.move(-.2);
            }
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        extender.move(0);
    }

    @Override
    public boolean isFinished() {
        return limitReached == true || ResetEncoders == false;
    }

    

    
}
