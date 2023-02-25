package frc.robot.commands.extender;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;
import frc.robot.utils.SmartShuffleboard;

public class ExtendToPosition extends CommandBase {
    private Extender extender;
    private double position;

    public ExtendToPosition(Extender extender, double position) {
        this.extender = extender;
        this.position = position;
        addRequirements(extender);
    }

    @Override
    public void end(boolean interrupted) {
        SmartShuffleboard.put("Arm","extenderState","ending");
        extender.stop();
    }

    @Override
    public void initialize() {
        SmartShuffleboard.put("Arm","extenderState","init");
    }

    @Override
    public void execute(){
        SmartShuffleboard.put("Arm","extenderState","executing");
        double speed = 0;
        double error = position - extender.getEncoder();
        if (Math.abs(error) > Constants.EXTENDER_LARGE_ERROR) {
            speed = Constants.EXTENDER_LARGE_ERROR_SPEED * Math.signum(error);
            extender.move(speed);
        }
        else {
            speed = Constants.EXTENDER_MINIMUM_SPEED * Math.signum(error);
            extender.move(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(extender.getEncoder()-position) < Constants.EXTENDER_ERROR_THRESHOLD);
    }
}
