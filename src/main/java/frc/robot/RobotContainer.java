// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmController;
import frc.robot.commands.Drive;
import frc.robot.commands.WheelAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ArmMotor;
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
  //private Drivetrain drivetrain;
  private ArmMotor armMotor;
  private PowerDistributionBoard m_PDB;
  private Joystick joyLeft = new Joystick(0);
  private Joystick joyRight = new Joystick(1);
  private CommandXboxController cmdController = new CommandXboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    //drivetrain = new Drivetrain();
    armMotor = new ArmMotor();
    m_PDB = new PowerDistributionBoard();

    configureBindings();
    //drivetrain.setDefaultCommand(new Drive(drivetrain, () -> joyLeft.getY(), () -> joyLeft.getX(), ()-> joyRight.getX()));
  }

  private void configureBindings() {
    cmdController.rightBumper().whileTrue(new ArmController(armMotor, Constants.ARM_CONTROLLER_CHANGE));
    cmdController.leftBumper().whileTrue(new ArmController(armMotor, -1 * Constants.ARM_CONTROLLER_CHANGE));
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

  //public Drivetrain getDrivetrain() {
  //  return drivetrain;
  //}

  public ArmMotor getArmMotor() {
    return armMotor;
  }

  public CommandXboxController getController() {
    return cmdController;
  }
}
