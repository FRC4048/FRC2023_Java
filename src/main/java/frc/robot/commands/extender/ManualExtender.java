package frc.robot.commands.extender;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;

public class ManualExtender extends CommandBase {
    private Extender extender;
    private boolean forward;
    private double speed;

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
    public void initialize() {
    }

    @Override
    public void execute(){
        if (forward) {
            speed = Constants.EXTENDER_MANUAL_SPEED;
        }
        else {
            speed = -Constants.EXTENDER_MANUAL_SPEED;
        }

        if (extender.getArm().getEncoderValue() <= 5) {
            speed = 0;
        }

        extender.move(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}