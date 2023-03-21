package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class CloseGripper extends LoggedCommand {
    private final GripperSubsystem gripper;
    private double startTime, prevTime;
    private double maxVelocity = 0;
    private double encoderPos;
    private double currentVelocity = 0;
    private Double grabStartTime = null;


    public CloseGripper(GripperSubsystem gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        gripper.stop();
    }

    @Override
    public void initialize() {
        super.initialize();
        maxVelocity = 0;
        currentVelocity = 0;
        startTime = Timer.getFPGATimestamp();
        grabStartTime = null;
        encoderPos = gripper.gripperPosition();
    }

    @Override
    public void execute() {
        gripper.close();

        currentVelocity = (Math.abs((gripper.gripperPosition() - encoderPos) / (Timer.getFPGATimestamp() - prevTime)));
        if (Math.abs(currentVelocity) > Math.abs(maxVelocity)) {
            maxVelocity = currentVelocity;
        }
        prevTime = Timer.getFPGATimestamp();
        encoderPos = gripper.gripperPosition();
        if (grabStartTime == null) {
            if (Math.abs(maxVelocity - currentVelocity) > .2) {
                grabStartTime = Timer.getFPGATimestamp();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (grabStartTime != null) {
            return (gripper.getClosedLimitSwitch()) || (Timer.getFPGATimestamp() - grabStartTime > Constants.WANTED_TIME);
        } else {
            return (gripper.getClosedLimitSwitch()) || (Timer.getFPGATimestamp() - startTime > Constants.GRIPPER_TIMEOUT);
        }
    }
}
