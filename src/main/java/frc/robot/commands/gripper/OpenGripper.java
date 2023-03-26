package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class OpenGripper extends LoggedCommand{
    private final GripperSubsystem gripper;

    private double startTime;

    public OpenGripper(GripperSubsystem gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        gripper.setHasPiece(false);
        gripper.stop();
    }

    @Override
    public void execute() {
        gripper.open();
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return (gripper.getopenLimitSwitch() == true || (Timer.getFPGATimestamp() - startTime) > Constants.GRIPPER_TIMEOUT);
    
}
}