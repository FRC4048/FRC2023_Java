// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.*;
import org.opencv.aruco.Aruco;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autonomous.MoveDistanceTraj;
import frc.robot.commands.arm.ArmMoveSequence;
import frc.robot.commands.arm.ManualMoveArm;
import frc.robot.commands.arm.MoveArmToGridPosition;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.TurnToDegrees;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.extender.ManualMoveExtender;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.ManualMoveGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.commands.sequences.GroundPickup;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.commands.sequences.StationPickupManual;
import frc.robot.commands.sequences.Stow;
import frc.robot.commands.sequences.SubstationPickup;
import frc.robot.subsystems.*;
import frc.robot.utils.SmartShuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ProtectionMechanism protectionMechanism;
  private Drivetrain drivetrain;
  private Arm arm;
  private Extender extender;
  private PowerDistributionBoard m_PDB;
  private GripperSubsystem gripper;
  private PieceGrid pieceGrid;
  private AutonomousChooser autonomousChooser;
  private PhotonCameraSubsystem photonSubsystem;


  //Joysticks & Joystick Buttons
  private Joystick joyLeft = new Joystick(Constants.LEFT_JOYSICK_ID);
  private Joystick joyRight = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  private JoystickButton LeftGyroButton= new JoystickButton(joyLeft, 1);
  private JoystickButton RightGyroButton= new JoystickButton(joyRight, 1);
  private JoystickButton joystickLeftButton = new JoystickButton(joyLeft, 2);
  private JoystickButton joystickLeftButton3 = new JoystickButton(joyLeft, 3);
  private JoystickButton joystickRightButton3 = new JoystickButton(joyRight, 3);



  //Xbox controllers
  private CommandXboxController manualController = new CommandXboxController(Constants.MANUAL_CONTROLLER_ID);
  private CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_ID);

  /*
  controller bindings:
    - a: grid select
    - b: stow
    - x: substation pickup
    - y: ground pickup

    - rTrigger: drop gamePiece
    - lTrigger: grab gamePiece

    - start: extend to position
    - logitechButton: abort alt
    - dPad: grid select
    - leftStickPress: station pickup

  manualController bindings:
  */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {


    // Configure the trigger bindings
    drivetrain = new Drivetrain();
    gripper = new GripperSubsystem();
    arm = new Arm();
    extender = new Extender();
    protectionMechanism = new ProtectionMechanism(arm,extender,gripper);

    autonomousChooser = new AutonomousChooser(drivetrain, arm, extender, gripper);
    autonomousChooser.addOptions();

    autonomousChooser.initialize();

    arm.setProtectionMechanism(protectionMechanism);
    extender.setProtectionMechanism(protectionMechanism);
    gripper.setProtectionMechanism(protectionMechanism);

    m_PDB = new PowerDistributionBoard();
    photonSubsystem = new PhotonCameraSubsystem();
    pieceGrid = new PieceGrid();
    configureBindings();
    putShuffleboardCommands();

    drivetrain.setDefaultCommand(new Drive(drivetrain, () -> joyLeft.getY(), () -> joyLeft.getX(), ()-> joyRight.getX(),joystickLeftButton));
  }



  private void configureBindings() {
    controller.povUpLeft().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.TOP_LEFT));
    controller.povLeft().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.MIDDLE_LEFT));
    controller.povDownLeft().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.DOWN_LEFT));
    controller.povUp().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.TOP_MIDDLE));
    controller.back().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.MIDDLE_MIDDLE));
    controller.povDown().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.DOWN_MIDDLE));
    controller.povUpRight().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.TOP_RIGHT));
    controller.povRight().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.MIDDLE_RIGHT));
    controller.povDownRight().onTrue(new SetGridSlot(pieceGrid, ArmPositionGrid.DOWN_RIGHT));
    LeftGyroButton.onTrue(new GyroOffseter(drivetrain, +5));
    RightGyroButton.onTrue(new GyroOffseter(drivetrain, -5));
    controller.button(XboxController.Button.kA.value).onTrue(new MoveArmToGridPosition(arm,extender,pieceGrid));
    controller.button(XboxController.Button.kLeftBumper.value).onTrue(new CloseGripper(gripper));
    controller.button(XboxController.Button.kRightBumper.value).onTrue(new OpenGripper(gripper));

    joystickLeftButton3.onTrue(new TurnToDegrees(drivetrain, 0));
    joystickRightButton3.onTrue(new TurnToDegrees(drivetrain, 180));

    manualController.button(XboxController.Button.kA.value).onTrue(new CloseGripper(gripper));
    controller.button(XboxController.Button.kB.value).onTrue(new Stow(arm, gripper, extender));
    controller.button(XboxController.Button.kY.value).onTrue(new GroundPickup(arm, extender, gripper));
    controller.button(XboxController.Button.kX.value).onTrue(new StationPickupManual(drivetrain, arm, extender, gripper));
    manualController.button(XboxController.Button.kX.value).whileTrue(new ManualMoveArm(arm, -Constants.MANUAL_ARM_SPEED));
    manualController.button(XboxController.Button.kY.value).whileTrue(new ManualMoveArm(arm, Constants.MANUAL_ARM_SPEED));
    manualController.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).onTrue(new ManualMoveGripper (gripper, () -> Constants.MANUAL_GRIP_SPEED ));
    manualController.axisLessThan(XboxController.Axis.kRightX.value, -0.1).onTrue(new ManualMoveGripper (gripper, () -> -Constants.MANUAL_GRIP_SPEED ));
    manualController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).onTrue(new ManualMoveExtender (extender, () -> Constants.MANUAL_EXTEND_SPEED ));
    manualController.axisLessThan(XboxController.Axis.kLeftY.value, -0.1).onTrue(new ManualMoveExtender (extender, () -> -Constants.MANUAL_EXTEND_SPEED ));

    controller.button(XboxController.Button.kLeftBumper.value).onTrue(new CloseGripper(gripper));
    controller.button(XboxController.Button.kRightBumper.value).onTrue(new OpenGripper(gripper));

    controller.button(XboxController.Button.kStart.value).onTrue(new CancelAll(drivetrain));

    extender.setDefaultCommand((new ManualMoveExtender(extender, () -> manualController.getLeftY())));
    gripper.setDefaultCommand(new ManualMoveGripper(gripper, () -> manualController.getLeftX()));

  }

  public void putShuffleboardCommands() {
    if (Constants.EXTENDER_DEBUG) {
      SmartShuffleboard.putCommand("Extender", "Set position=5709", new ExtendToPosition(extender, 5709));
      SmartShuffleboard.putCommand("Extender", "Set position=4000", new ExtendToPosition(extender, 4000));
<<<<<<< HEAD
    SmartShuffleboard.putCommand("Extender", "Stow", new Stow(arm, gripper, extender));
    SmartShuffleboard.putCommand("Driver", "Turn to Degee", new TurnToDegrees(drivetrain, 45));
    
      SmartShuffleboard.putCommand("Arm", "Manual UP", new ManualMoveArm(arm, 3.0));
      SmartShuffleboard.putCommand("Arm", "Manual DOWN", new ManualMoveArm(arm, -1.5));
      SmartShuffleboard.putCommand("Arm", "GO TO 10", new ArmMoveSequence(arm,extender,10,0));
      SmartShuffleboard.putCommand("Arm", "GO TO 15", new ArmMoveSequence(arm,extender,15,0));
      SmartShuffleboard.putCommand("Arm", "GO TO 25", new ArmMoveSequence(arm,extender,25,0));
      SmartShuffleboard.putCommand("Drive", "ResetGyro", new ResetGyro(getDrivetrain(), 0));
      SmartShuffleboard.putCommand("Driver", "MoveDistance", new MoveDistanceTraj(drivetrain, 0.5, 0.5));
  
=======
      SmartShuffleboard.putCommand("Extender", "Stow", new Stow(arm, gripper, extender));
>>>>>>> e5ac88e7458a6a0207b7576a7b2290239493782b
      SmartShuffleboard.putCommand("Extender", "Reset Encoders (Arm and Extender)", new ResetEncoders(arm, extender));
    }

    if (Constants.ARM_DEBUG) {
      SmartShuffleboard.putCommand("Driver", "Cross", new CrossPanel(drivetrain));
    SmartShuffleboard.putCommand("Arm", "Manual UP", new ManualMoveArm(arm, 3.0));
    SmartShuffleboard.putCommand("Arm", "Manual DOWN", new ManualMoveArm(arm, -1.5));SmartShuffleboard.putCommand("Arm", "GO TO 10", new ArmMoveSequence(arm,extender,10,0));
      SmartShuffleboard.putCommand("Arm", "GO TO 15", new ArmMoveSequence(arm,extender,15,0));
      SmartShuffleboard.putCommand("Arm", "GO TO 25", new ArmMoveSequence(arm,extender,25,0));
    }
    SmartShuffleboard.putCommand("Drive", "ResetGyro", new ResetGyro(getDrivetrain(), 0));
    SmartShuffleboard.putCommand("Driver", "MoveDistance", new MoveDistanceTraj(drivetrain, 0.5, 0.5));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


   public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonomousChooser.getAutonomousCommand();
  }

  // return new SequentialCommandGroup(
  //   new MoveDistanceSpinTraj(drivetrain, 0.5, 0.30, Math.toRadians(180)),
  //   new MoveDistanceSpinTraj(drivetrain, 4.35, 0.35, Math.toRadians(0))
  //   );

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Arm getArm() {
    return arm;
  }

  public GripperSubsystem getGripper() {
    return gripper;
  }


  public CommandXboxController getController() {
    return controller;
  }

  public Extender getExtender() {
    return extender;
  }


  public Joystick getJoyLeft() {
    return joyLeft;
  }


  public AutonomousChooser getAutonomousChooser() {
    return autonomousChooser;
  }

}

