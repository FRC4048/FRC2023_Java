package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Logger;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ResetArm extends LoggedCommand{

    private Arm arm;
    private boolean limitReached;
    private double startTime;

    public ResetArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        limitReached = false;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        boolean revLimit = arm.isRevLimitSwitchReached();
            if (revLimit) {
                arm.resetEncoder();
                limitReached = true;
            } else {
            arm.setVoltage(-2.0);
        }
    }

    @Override
    public boolean isFinished() {
        if (limitReached) {
            return true;
        }
        if ((Timer.getFPGATimestamp() - startTime) > Constants.ARM_RESET_TIMEOUT) {
            Logger.logTimeout(getName(), Constants.ENABLE_LOGGING);
            return true;
        }
        return false;
    }
}
