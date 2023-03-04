// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.GyroOffseter;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetGridSlot;
import frc.robot.commands.SubstationTrajAllign;
import frc.robot.commands.Autonomous.MoveDistanceSpinTraj;
import frc.robot.commands.arm.ManualMoveArm;
import frc.robot.commands.arm.MoveArmToGridPosition;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.MoveDistanceX;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.extender.ManualMoveExtender;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.ManualMoveGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.commands.sequences.GroundPickup;
import frc.robot.commands.sequences.StationPickupAuto;
import frc.robot.commands.sequences.StationPickupManual;
import frc.robot.commands.sequences.Stow;
import frc.robot.subsystems.AprilTagPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PieceGrid;
import frc.robot.subsystems.PowerDistributionBoard;
import frc.robot.subsystems.ProtectionMechanism;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.luxonis.LuxonisVision;

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
  private LuxonisVision luxonisVision;
  private Arm arm;
  private Extender extender;
  private PowerDistributionBoard m_PDB;
  private GripperSubsystem gripper;
  private AprilTagPosition aprilTagPosition;
  private PieceGrid pieceGrid;
  private AutonomousChooser autonomousChooser;


  //Joysticks & Joystick Buttons
  private Joystick joyLeft = new Joystick(Constants.LEFT_JOYSICK_ID);
  private Joystick joyRight = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  private JoystickButton LeftGyroButton= new JoystickButton(joyLeft, 1);
  private JoystickButton RightGyroButton= new JoystickButton(joyRight, 1);
  private JoystickButton joystickLeftButton = new JoystickButton(joyLeft, 2);

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
    luxonisVision = new LuxonisVision();
    aprilTagPosition = new AprilTagPosition();
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

    manualController.button(XboxController.Button.kA.value).onTrue(new CloseGripper(gripper));
    controller.button(XboxController.Button.kB.value).onTrue(new Stow(arm, gripper, extender));
    controller.button(XboxController.Button.kY.value).onTrue(new GroundPickup(arm, extender, gripper));
    controller.button(XboxController.Button.kX.value).onTrue(new StationPickupAuto(arm, gripper, drivetrain, luxonisVision, extender));
    manualController.button(XboxController.Button.kX.value).whileTrue(new ManualMoveArm(arm, -Constants.MANUAL_ARM_SPEED));
    manualController.button(XboxController.Button.kY.value).whileTrue(new ManualMoveArm(arm, Constants.MANUAL_ARM_SPEED));
    manualController.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).onTrue(new ManualMoveGripper (gripper, () -> Constants.MANUAL_GRIP_SPEED ));
    manualController.axisLessThan(XboxController.Axis.kRightX.value, -0.1).onTrue(new ManualMoveGripper (gripper, () -> -Constants.MANUAL_GRIP_SPEED ));
    manualController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).onTrue(new ManualMoveExtender (extender, () -> Constants.MANUAL_EXTEND_SPEED ));
    manualController.axisLessThan(XboxController.Axis.kLeftY.value, -0.1).onTrue(new ManualMoveExtender (extender, () -> -Constants.MANUAL_EXTEND_SPEED ));

    controller.button(XboxController.Button.kLeftBumper.value).onTrue(new CloseGripper(gripper));
    controller.button(XboxController.Button.kRightBumper.value).onTrue(new OpenGripper(gripper));
   
    extender.setDefaultCommand((new ManualMoveExtender(extender, () -> manualController.getLeftY())));
    gripper.setDefaultCommand(new ManualMoveGripper(gripper, () -> manualController.getLeftX()));
  }

  public void putShuffleboardCommands() {

    if (Constants.EXTENDER_DEBUG) {
      SmartShuffleboard.putCommand("Extender", "Set position=5709", new ExtendToPosition(extender, 5709));
      SmartShuffleboard.putCommand("Extender", "Set position=4000", new ExtendToPosition(extender, 4000));
      SmartShuffleboard.putCommand("Extender", "Stow", new Stow(arm, gripper, extender));
    }
    if (Constants.ARM_DEBUG) {
      SmartShuffleboard.putCommand("Arm", "Manual UP", new ManualMoveArm(arm, 3.0));
      SmartShuffleboard.putCommand("Arm", "Manual DOWN", new ManualMoveArm(arm, -1.5));
    }
    
    SmartShuffleboard.putCommand("Drive", "Reset Gyro", new ResetGyro(getDrivetrain(), 0));
    SmartShuffleboard.putCommand("Drive", "Reset Encoders", new ResetEncoders(arm, extender));
    SmartShuffleboard.putCommand("Substation", "manual move back", new MoveDistanceX(drivetrain, -0.63));
    SmartShuffleboard.putCommand("Substation", "Traj ALLIGN", new SubstationTrajAllign(drivetrain, luxonisVision, 0.83));
    SmartShuffleboard.putCommand("Substation", "auto pickup", new StationPickupAuto(arm, gripper, drivetrain, luxonisVision, extender));
    SmartShuffleboard.putCommand("Substation", "manual pickup", new StationPickupManual(drivetrain, arm, extender, gripper));
    SmartShuffleboard.putCommand("Substation", "move back", new MoveDistanceSpinTraj(drivetrain, 0.63, 0, Math.toDegrees(180)));
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

