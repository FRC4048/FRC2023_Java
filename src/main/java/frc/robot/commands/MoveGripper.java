package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;

public class MoveGripper extends CommandBase {

    private GripperSubsystem gripper;
    private double startTime;
    public MoveGripper (GripperSubsystem gripper) {
        this.gripper=gripper;
        addRequirements(gripper);
    }
    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        gripper.open();
    }

    @Override
    public boolean isFinished() {
        return (gripper.getopenLimitSwitch() == true || (Timer.getFPGATimestamp() - startTime) > 2);
    }
}
