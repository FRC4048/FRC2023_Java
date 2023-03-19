package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ResetArm extends CommandBase{

    private Arm arm;
    private boolean limitReached;
    private double startTime;

    public ResetArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void initialize() {
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
        return limitReached || Timer.getFPGATimestamp() - startTime > Constants.ARM_RESET_TIMEOUT;
    }
    
}
