package frc.robot.commands.Autonomous;

import frc.robot.commands.Forward;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Cone extends SequentialCommandGroup {
  public Cone(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    addCommands(new Forward(drivetrain));
  }
}