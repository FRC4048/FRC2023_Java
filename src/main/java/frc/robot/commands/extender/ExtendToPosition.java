package frc.robot.commands.extender;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.utils.SmartShuffleboard;

public class ExtendToPosition extends CommandBase {
    private Extender extender;
    private double position;
    private long startTime;
    private final long timeout = 5000;

    public ExtendToPosition(Extender extender, double position) {
        this.extender = extender;
        this.position = position;
        addRequirements(extender);
    }

    @Override
    public void end(boolean interrupted) {
        extender.stop();
        if (extender.revLimitReached()) extender.resetEncoder();
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        double speed;
        if (System.currentTimeMillis() - startTime >= timeout)end(true);
        double error = position - extender.getEncoder();
        if (Math.abs(error) > Constants.EXTENDER_SPEED_SLOW_THRESHOLD) {
            speed = Constants.EXTENDER_AUTO_MAX_SPEED * Math.signum(error);
            extender.move(speed);
        } else {
            double targetRange = Constants.EXTENDER_AUTO_MAX_SPEED - Constants.EXTENDER_AUTO_MIN_SPEED;
            speed = (error/ Constants.EXTENDER_SPEED_SLOW_THRESHOLD) //range is now 0-1
                    * (targetRange) // range is now in target range shifted down by the min range
                    + (Constants.EXTENDER_AUTO_MIN_SPEED *Math.signum(error)); // shift range to target range
            extender.move(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(extender.getEncoder()-position) < Constants.EXTENDER_DESTINATION_THRESHOLD);
    }
}
