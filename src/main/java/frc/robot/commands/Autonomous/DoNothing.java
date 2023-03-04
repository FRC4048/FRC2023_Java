package frc.robot.commands.Autonomous;

import frc.robot.commands.ResetEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class DoNothing extends SequentialCommandGroup {
  public DoNothing(Arm arm, Extender extender) {
    addCommands(new ResetEncoders(arm, extender));
    }
}
