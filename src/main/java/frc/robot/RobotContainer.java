// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.commands.NeoDrive;
import frc.robot.commands.TalonDrive;
import frc.robot.commands.WheelAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NeoSubsystem;
import frc.robot.subsystems.PowerDistributionBoard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.TalonSubsystem;
import frc.robot.utils.SmartShuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain;
  private PowerDistributionBoard m_PDB;
  private Joystick joyLeft = new Joystick(0);
  private Joystick joyRight = new Joystick(1);
  private JoystickButton button_1= new JoystickButton(joyLeft, 1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private NeoSubsystem neo1;
  private TalonSubsystem talon1;
  private TalonSubsystem talon2;

  public RobotContainer() {
//    drivetrain = new Drivetrain();
    m_PDB = new PowerDistributionBoard();

//    drivetrain.setDefaultCommand(new Drive(drivetrain, () -> joyLeft.getY(), () -> joyLeft.getX(), ()-> joyRight.getX()));

    neo1 = new NeoSubsystem("Neo1", Constants.NEO_TEST_1);
    talon1 = new TalonSubsystem("Talon1", Constants.TALON_TEST_1);
    talon2 = new TalonSubsystem("Talon2", Constants.TALON_TEST_2);

    configureBindings();
  }

  private void configureBindings() {
  //  button_1.onTrue(new WheelAlign(drivetrain));

    SmartShuffleboard.putCommand(Constants.TEST_MOTOR, "NeoDrive",  new NeoDrive("Neo drive", getNeo1()));
    SmartShuffleboard.putCommand(Constants.TEST_MOTOR, "Talon1Drive",  new TalonDrive("Talon Drive 1", getTalon1()));
    SmartShuffleboard.putCommand(Constants.TEST_MOTOR, "Talon2Drive",  new TalonDrive("Talon Drive 2", getTalon2()));
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

  public NeoSubsystem getNeo1() {
    return neo1;
  }

  public TalonSubsystem getTalon1() {
    return talon1;
  }

  public TalonSubsystem getTalon2() {
    return talon2;
  }
}
