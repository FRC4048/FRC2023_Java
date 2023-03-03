package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.sequences.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class Cancel extends SequentialCommandGroup{

    public Cancel(Arm arm, Extender extender, GripperSubsystem gripper) {
        CommandScheduler.getInstance().cancelAll();
        addRequirements(arm, gripper, extender);
        new Stow(arm, gripper, extender);
    }
    
}
