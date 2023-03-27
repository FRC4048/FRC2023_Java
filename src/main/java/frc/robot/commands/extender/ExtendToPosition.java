package frc.robot.commands.extender;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;
import frc.robot.utils.Logger;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ExtendToPosition extends LoggedCommand {
    private Extender extender;
    private double position;
    private double startTime;

    public ExtendToPosition(Extender extender, double position) {
        this.extender = extender;
        this.position = position;
        addRequirements(extender);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        extender.stop();
        if (extender.revLimitReached()) extender.resetEncoder();
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        double speed;
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
        if (Math.abs(extender.getEncoder() - position) < Constants.EXTENDER_DESTINATION_THRESHOLD) {
            return true;
        }
        if ((Timer.getFPGATimestamp() - startTime) > Constants.EXTEND_TO_POSITION_TIMEOUT) {
            Logger.logTimeout(getName(), Constants.ENABLE_LOGGING);
            return true;
        }
        return false;
    }
}
