package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MatrixSetter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drivetrain drivetrain;
  private static XboxController controller = new XboxController(2);

  private static int[] POV = {-1, 0, 45, 90, 135, 180, 225, 270, 315};

  public MatrixSetter(Drivetrain drivetrain, XboxController controller) { 
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
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
  public boolean isFinished() {
    return true;
  }
}
