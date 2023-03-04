package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class CloseGripper extends CommandBase {
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
        gripper.stop();
    }

    @Override
    public void initialize() {
        maxVelocity = 0;
        currentVelocity = 0;
        startTime = Timer.getFPGATimestamp();
        grabStartTime = null;

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
        if (Math.abs(currentVelocity - maxVelocity) > .2) {
            grabStartTime = Timer.getFPGATimestamp();
        }

    }

    @Override
    public boolean isFinished() {
        if (grabStartTime != null) {
            return Timer.getFPGATimestamp() - grabStartTime > Constants.WANTED_TIME; 
        }
        else {
            return (Timer.getFPGATimestamp() - startTime > Constants.GRIPPER_TIMEOUT);
        
        }
    }
}
