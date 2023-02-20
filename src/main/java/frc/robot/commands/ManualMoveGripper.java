package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class ManualMoveGripper extends CommandBase{
    private final GripperSubsystem gripper;
    private final DoubleSupplier dSupplier;
    public ManualMoveGripper(GripperSubsystem gripper, DoubleSupplier dSupplier) {
        this.gripper = gripper;
        addRequirements(gripper);
        this.dSupplier = dSupplier;
    }
    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }

    @Override
    public void execute() {
        double speed = dSupplier.getAsDouble();
        // Scaling by half to ensure the gripper does not move too quickly
        speed = 0.5*speed;
        gripper.move(speed);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
