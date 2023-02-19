// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.ExtenderManual;
import frc.robot.commands.ExtenderMovePos;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.utils.ExtenderPosition;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should ,.be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
  private ExtenderPosition position = ExtenderPosition.RETRACT_FULL;
  private final CommandXboxController extenderController = new CommandXboxController(2);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    extenderSubsystem.resetEncoder();
    new ExtenderMovePos(extenderSubsystem,ExtenderPosition.RETRACT_FULL);
    extenderController.x().whileTrue(new ExtenderMovePos(extenderSubsystem, ExtenderPosition.RETRACT_FULL));
    extenderController.a().whileTrue(new ExtenderMovePos(extenderSubsystem, ExtenderPosition.BOTTOM_ROW));
    extenderController.b().whileTrue(new ExtenderMovePos(extenderSubsystem, ExtenderPosition.MIDDLE_ROW));
    extenderController.y().whileTrue(new ExtenderMovePos(extenderSubsystem, ExtenderPosition.TOP_ROW));
    extenderController.leftTrigger().whileTrue(new ExtenderManual(extenderSubsystem,false));
    extenderController.rightTrigger().whileTrue(new ExtenderManual(extenderSubsystem,true));
    SmartShuffleboard.put("test", "set pos", position.getPosition());
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public CommandXboxController getController() {
    return extenderController;
  }
}
