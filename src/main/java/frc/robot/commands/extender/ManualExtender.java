package frc.robot.commands.extender;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;

public class ManualExtender extends CommandBase {
    private Extender extender;
    private boolean forward;

    public ManualExtender(Extender extender, boolean forward) {
        this.extender = extender;
        this.forward = forward;
        addRequirements(extender);
    }

    @Override
    public void end(boolean interrupted) {
        extender.stop();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        if (forward) {
            extender.move(Constants.EXTENDER_MANUAL_SPEED);
        }
        else {
            extender.move(-Constants.EXTENDER_MANUAL_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}