// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CancelAll;
import frc.robot.commands.ChangeLedID;
import frc.robot.commands.CrossPanel;
import frc.robot.commands.CycleLED;
import frc.robot.commands.GyroOffseter;
import frc.robot.commands.SetGridSlot;
import frc.robot.commands.SetLEDID;
import frc.robot.commands.arm.ManualMoveArm;
import frc.robot.commands.arm.MoveArmToGridPosition;
import frc.robot.commands.drive.AlignToGrid;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.extender.ManualMoveExtender;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LedPanel;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.PieceGrid;
import frc.robot.subsystems.PowerDistributionBoard;
import frc.robot.subsystems.ProtectionMechanism;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;

import java.util.function.DoubleSupplier;

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
  private LedPanel ledPanel;
  private PowerDistributionBoard m_PDB;
  private GripperSubsystem gripper;
  private PieceGrid pieceGrid;
  private AutonomousChooser autonomousChooser;
  private CycleLED disabledCycleLED;
  private CycleLED autoCycleLED;
  private CycleLED testCycleLED;
  private PhotonCameraSubsystem photonSubsystem;


  //Joysticks & Joystick Buttons
  private Joystick joyLeft = new Joystick(Constants.LEFT_JOYSICK_ID);
  private Joystick joyRight = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  private JoystickButton LeftGyroButton= new JoystickButton(joyLeft, 1);
  private JoystickButton RightGyroButton= new JoystickButton(joyRight, 1);
  private JoystickButton joystickLeftButton = new JoystickButton(joyLeft, 2);
  private JoystickButton joystickRightButton = new JoystickButton(joyRight, 2);
  private JoystickButton joystickLeftButton3 = new JoystickButton(joyLeft, 3);
  private JoystickButton joystickLeftButton14 = new JoystickButton(joyRight, 14);
  private JoystickButton joystickLeftButton16 = new JoystickButton(joyRight, 16);

  //Xbox controllers
  private CommandXboxController manualController = new CommandXboxController(Constants.MANUAL_CONTROLLER_ID);
  private CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_ID);
  double armSubstationOffset = 0;
  private DoubleSupplier armSubOffset = () -> armSubstationOffset;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {


    // Configure the trigger bindings
    photonSubsystem = new PhotonCameraSubsystem();
    drivetrain = new Drivetrain(photonSubsystem);
    gripper = new GripperSubsystem();
    arm = new Arm();
    extender = new Extender();
    ledPanel = new LedPanel();
    protectionMechanism = new ProtectionMechanism(arm,extender,gripper);
    testCycleLED = new CycleLED(ledPanel,1,1,2,3,4,5,6,7);
    disabledCycleLED = new CycleLED(ledPanel,1,4);
    autoCycleLED = new CycleLED(ledPanel,1,7);
    autonomousChooser = new AutonomousChooser(drivetrain, arm, extender, gripper);
    autonomousChooser.addOptions();

    autonomousChooser.initialize();

    arm.setProtectionMechanism(protectionMechanism);
    extender.setProtectionMechanism(protectionMechanism);
    gripper.setProtectionMechanism(protectionMechanism);

//    m_PDB = new PowerDistributionBoard();
    pieceGrid = new PieceGrid();
    configureBindings();
    putShuffleboardCommands();

    drivetrain.setDefaultCommand(new Drive(drivetrain, () -> joyLeft.getY(), () -> joyLeft.getX(), ()-> joyRight.getX(),joystickLeftButton, joystickRightButton));

    UsbCamera camera = CameraServer.startAutomaticCapture(0);

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

    //controls
    controller.button(XboxController.Button.kA.value).onTrue(new MoveArmToGridPosition(arm,extender,pieceGrid));
    controller.button(XboxController.Button.kB.value).onTrue(new SequentialCommandGroupWrapper(new Stow(arm, gripper, extender), "-Button-Stow"));
//    controller.button(XboxController.Button.kY.value).onTrue(new SequentialCommandGroupWrapper(new GroundPickup(arm, extender, gripper), "-Button-Ground-Pickup"));
    controller.button(XboxController.Button.kX.value).onTrue(new SequentialCommandGroupWrapper(new SubstationAutoPickup(arm, gripper, extender,armSubOffset), "-Button-Substation-Pickup"));
    controller.button(XboxController.Button.kY.value).onTrue(new SequentialCommandGroupWrapper(new SubstationAutoPickupWithMove(arm, gripper,drivetrain,armSubOffset), "-Button-Substation-Pickup"));


    controller.button(XboxController.Button.kLeftBumper.value).onTrue(new OpenGripper(gripper));
    controller.button(XboxController.Button.kRightBumper.value).onTrue(new CloseGripper(gripper));
    controller.button(XboxController.Button.kStart.value).onTrue(new CancelAll(drivetrain));

    //arm move on stick
    controller.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1).whileTrue(new ManualMoveArm(arm, -Constants.MANUAL_ARM_SPEED));
    controller.axisLessThan(XboxController.Axis.kRightY.value, -0.1).whileTrue(new ManualMoveArm(arm, Constants.MANUAL_ARM_SPEED));

    //extender move on stick
    controller.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.1).whileTrue(new ManualMoveExtender(extender, () -> -Constants.EXTENDER_MANUAL_SPEED));
    controller.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.1).whileTrue(new ManualMoveExtender(extender, () -> Constants.EXTENDER_MANUAL_SPEED));

    //led panel
    controller.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).onTrue(new SetLEDID(ledPanel, Constants.CONE_ID));
    controller.axisLessThan(XboxController.Axis.kLeftY.value, -0.1).onTrue(new SetLEDID(ledPanel, Constants.CUBE_ID));
    controller.button(XboxController.Button.kLeftStick.value).onTrue(new SetLEDID(ledPanel, Constants.ROBOT_ID));
    controller.button(XboxController.Button.kRightStick.value).onTrue(new ChangeLedID(ledPanel, 1));

    manualController.button(XboxController.Button.kLeftBumper.value).onTrue(new CloseGripper(gripper));
    manualController.button(XboxController.Button.kRightBumper.value).onTrue(new OpenGripper(gripper));
    manualController.button(XboxController.Button.kX.value).whileTrue(new ManualMoveArm(arm, -Constants.MANUAL_ARM_SPEED));
    manualController.button(XboxController.Button.kY.value).whileTrue(new ManualMoveArm(arm, Constants.MANUAL_ARM_SPEED));

    extender.setDefaultCommand((new ManualMoveExtender(extender, () -> manualController.getLeftY())));

    LeftGyroButton.onTrue(new GyroOffseter(drivetrain, +5));
    RightGyroButton.onTrue(new GyroOffseter(drivetrain, -5));
    
    joystickLeftButton3.onTrue(new AlignToGrid(drivetrain));

    joystickLeftButton14.onTrue(new InstantCommand(()-> {
      armSubstationOffset -=(.25 / Constants.ARM_ENCODER_CONVERSION_FACTOR);
      Logger.logDouble("/arm/substationOffset",armSubstationOffset,Constants.ENABLE_LOGGING);
    }));
    joystickLeftButton16.onTrue(new InstantCommand(()-> {
      armSubstationOffset += (.25 / Constants.ARM_ENCODER_CONVERSION_FACTOR);
      Logger.logDouble("/arm/substationOffset",armSubstationOffset,Constants.ENABLE_LOGGING);
    }));
  }

  public void putShuffleboardCommands() {
    if (Constants.EXTENDER_DEBUG) {
      SmartShuffleboard.putCommand("Extender", "Set position=5709", new ExtendToPosition(extender, 5709));
      SmartShuffleboard.putCommand("Extender", "Set position=4000", new ExtendToPosition(extender, 4000));
      SmartShuffleboard.putCommand("Extender", "Stow", new SequentialCommandGroupWrapper(new Stow(arm, gripper, extender),"-Debug-Stow"));
    }

    if (Constants.ARM_DEBUG) {
      SmartShuffleboard.putCommand("Arm", "Manual UP", new ManualMoveArm(arm, 3.0));
      SmartShuffleboard.putCommand("Arm", "Manual DOWN", new ManualMoveArm(arm, -1.5));
      SmartShuffleboard.putCommand("Arm","SetpointDebugExecute",new SetpointDebug(arm));
      SmartShuffleboard.put("Arm","SetpointDebugValue",0);
    }
    if (Constants.DRIVETRAIN_DEBUG) {
      SmartShuffleboard.putCommand("Drive", "Cross", new CrossPanel(drivetrain));
      SmartShuffleboard.putCommand("Drive", "Cross+Balance", new CrossAndBalance(drivetrain));
    }
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

  public LedPanel getLedPanel() {
    return ledPanel;
  }

  public CycleLED getDisabledLedCycleCommand() {
    return disabledCycleLED;
  }
  public CycleLED getAutoLedCycleCommand() {
    return autoCycleLED;
  }
  public CycleLED getTestLedCycleCommand() {
    return testCycleLED;
  }
}

