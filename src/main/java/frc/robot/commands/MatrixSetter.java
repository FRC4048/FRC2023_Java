package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MatrixSetter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ExampleSubsystem subsystem;
  private static XboxController controller;

  private static int[] POV = {-1, 0, 45, 90, 135, 180, 225, 270, 315};

  public MatrixSetter(ExampleSubsystem subsystem) { 
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  public static boolean isCurrentState(int state) {
    if(controller.getPOV() == POV[state]) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void initialize() {
    controller = new XboxController(0);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
