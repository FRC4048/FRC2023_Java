// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Turn;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public AutonomousChooser autonomousChooser = new AutonomousChooser(m_exampleSubsystem);
  public LocationChooser locationChooser = new LocationChooser(m_exampleSubsystem);


  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autonomousChooser.addOptions();
    locationChooser.addOptions();

    autonomousChooser.initialize();
    locationChooser.initialize();
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    Turn turn = new Turn(m_exampleSubsystem, () -> m_driverController.getLeftY());
    m_exampleSubsystem.setDefaultCommand(turn);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonomousChooser.getAutonomousCommand(autonomousChooser.getAction());
  }

  public int getLocation() {
    return locationChooser.getLocation(locationChooser.getAction());
  }
}
