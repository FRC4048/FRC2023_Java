package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class GoToSetpoint extends CommandBase {
    
    private Arm arm;
    private double angle;

    public GoToSetpoint(Arm arm) {
        angle = arm.getAngle();
        new ArmMoveSequence(arm, angle).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
