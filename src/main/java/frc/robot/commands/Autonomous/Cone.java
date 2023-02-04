
package frc.robot.commands.Autonomous;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class Cone extends SequentialCommandGroup {

  public Cone(ExampleSubsystem subsystem) {
    addRequirements(subsystem);
    addCommands(new ExampleCommand(subsystem));
  }
}