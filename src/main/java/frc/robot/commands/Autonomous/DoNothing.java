package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;

/** An example command that uses an example subsystem. */
public class DoNothing extends SequentialCommandGroup {
  public DoNothing(Arm arm, Extender extender, Drivetrain drivetrain) {
    addCommands(
    new InstantCommand(drivetrain::stopMotors, drivetrain),
    new ResetEncoders(arm, extender),
    new InstantCommand(drivetrain::stopMotors, drivetrain)
    );
    }
}
