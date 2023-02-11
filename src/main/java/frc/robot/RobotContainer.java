package frc.robot;

import frc.robot.commands.MatrixSetter;
import frc.robot.commands.matrixCommands;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController cmdController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    cmdController.povUp().onTrue(new matrixCommands(exampleSubsystem, "up"));
    cmdController.povUpRight().onTrue(new matrixCommands(exampleSubsystem, "upRight"));
    cmdController.povRight().onTrue(new matrixCommands(exampleSubsystem, "right"));
    cmdController.povDownRight().onTrue(new matrixCommands(exampleSubsystem, "downRight"));
    cmdController.povDown().onTrue(new matrixCommands(exampleSubsystem, "down"));
    cmdController.povDownLeft().onTrue(new matrixCommands(exampleSubsystem, "downLeft"));
    cmdController.povLeft().onTrue(new matrixCommands(exampleSubsystem, "left"));
    cmdController.povUpLeft().onTrue(new matrixCommands(exampleSubsystem, "upLeft"));
    cmdController.povCenter().onTrue(new matrixCommands(exampleSubsystem, "still"));
  }

  public CommandBase matrixSetState() {
    return new MatrixSetter(exampleSubsystem);
  }
}
