package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;

public class OpenGripper extends CommandBase{
    private final GripperSubsystem gripper;

    private double startTime;



    public OpenGripper(GripperSubsystem gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }
    @Override
    public void end(boolean interrupted) {
        gripper.setHasPiece(false);
        gripper.stop();
    }

    @Override
    public void execute() {
        gripper.open();
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return (gripper.getopenLimitSwitch() == true || (Timer.getFPGATimestamp() - startTime) > Constants.GRIPPER_TIMEOUT);
    
}
}