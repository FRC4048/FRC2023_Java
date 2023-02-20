// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmController;
import frc.robot.commands.Drive;
import frc.robot.commands.WheelAlign;
import frc.robot.commands.MatrixCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PowerDistributionBoard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain;
  private Arm arm;
  private PowerDistributionBoard m_PDB;
  private Joystick joyLeft = new Joystick(0);
  private Joystick joyRight = new Joystick(1);
  private CommandXboxController cmdController = new CommandXboxController(2);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    drivetrain = new Drivetrain();
    arm = new Arm();
    m_PDB = new PowerDistributionBoard();

    configureBindings();
    drivetrain.setDefaultCommand(new Drive(drivetrain, () -> joyLeft.getY(), () -> joyLeft.getX(), ()-> joyRight.getX()));
  }

  private void configureBindings() {
    cmdController.rightBumper().whileTrue(new ArmController(arm, Constants.ARM_CONTROLLER_CHANGE));
    cmdController.leftBumper().whileTrue(new ArmController(arm, -1 * Constants.ARM_CONTROLLER_CHANGE));
 
    cmdController.povUp().onTrue(new MatrixCommands(drivetrain, arm, "up"));
    cmdController.povUpRight().onTrue(new MatrixCommands(drivetrain, arm, "upRight"));
    cmdController.povRight().onTrue(new MatrixCommands(drivetrain, arm, "right"));
    cmdController.povDownRight().onTrue(new MatrixCommands(drivetrain, arm, "downRight"));
    cmdController.povDown().onTrue(new MatrixCommands(drivetrain, arm, "down"));
    cmdController.povDownLeft().onTrue(new MatrixCommands(drivetrain, arm, "downLeft"));
    cmdController.povLeft().onTrue(new MatrixCommands(drivetrain, arm, "left"));
    cmdController.povUpLeft().onTrue(new MatrixCommands(drivetrain, arm, "upLeft"));
    cmdController.povCenter().onTrue(new MatrixCommands(drivetrain, arm, "still"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("hi");
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Arm getArm() {
    return arm;
  }

  public CommandXboxController getController() {
    return cmdController;
  }
}
