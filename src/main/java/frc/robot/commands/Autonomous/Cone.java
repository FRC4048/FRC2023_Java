
package frc.robot.commands.Autonomous;

import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class Cone extends CommandBase {

  public Cone(ExampleSubsystem subsystem) {
    addRequirements(subsystem);
  }
}