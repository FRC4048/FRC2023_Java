
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

/** An example command that uses an example subsystem. */
public class Cone extends SequentialCommandGroup {

  public Cone() {
    
    addCommands(new EmptyCommand());
  }
}