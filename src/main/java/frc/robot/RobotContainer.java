// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.GyroOffseter;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.Stow;
import frc.robot.commands.arm.ArmMoveSequence;
import frc.robot.commands.arm.ManualMoveArm;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.Forward;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.extender.ManualMoveExtender;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.ManualMoveGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PowerDistributionBoard;
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
  private Arm arm;
  private Extender extender;
  private PowerDistributionBoard m_PDB;
  private GripperSubsystem gripper;

  //Joysticks & Joystick Buttons
  private Joystick joyLeft = new Joystick(Constants.LEFT_JOYSICK_ID);
  private Joystick joyRight = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  private JoystickButton LeftGyroButton= new JoystickButton(joyLeft, 1);
  private JoystickButton RightGyroButton= new JoystickButton(joyRight, 1);

  //Xbox controllers
  private CommandXboxController xboxManual = new CommandXboxController(4);
  private XboxController xboxAuto = new XboxController(3);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    drivetrain = new Drivetrain();
    gripper = new GripperSubsystem();
    arm = new Arm();
    extender = new Extender();
    m_PDB = new PowerDistributionBoard();

    configureBindings();
    putShuffleboardCommands();
    drivetrain.setDefaultCommand(new Drive(drivetrain, () -> joyLeft.getY(), () -> joyLeft.getX(), ()-> joyRight.getX()));
  }

  private void configureBindings() {
    LeftGyroButton.onTrue(new GyroOffseter(drivetrain, -1));
    RightGyroButton.onTrue(new GyroOffseter(drivetrain, +1));

    xboxManual.button(XboxController.Button.kA.value).onTrue(new CloseGripper(gripper));
    xboxManual.button(XboxController.Button.kB.value).onTrue(new OpenGripper(gripper));
    xboxManual.button(4).whileTrue(new ManualMoveArm(arm, Constants.MANUAL_ARM_SPEED));
    xboxManual.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).onTrue(new ManualMoveGripper (gripper, () -> Constants.MANUAL_GRIP_SPEED ));
    xboxManual.axisLessThan(XboxController.Axis.kRightX.value, -0.1).onTrue(new ManualMoveGripper (gripper, () -> -Constants.MANUAL_GRIP_SPEED ));
    xboxManual.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).onTrue(new ManualMoveExtender (extender, () -> Constants.MANUAL_EXTEND_SPEED ));
    xboxManual.axisLessThan(XboxController.Axis.kLeftY.value, -0.1).onTrue(new ManualMoveExtender (extender, () -> -Constants.MANUAL_EXTEND_SPEED ));

  }

  public void putShuffleboardCommands() {

    if (Constants.DEBUG) {
    SmartShuffleboard.putCommand("Extender", "Set position=5709", new ExtendToPosition(extender, 5709));
    SmartShuffleboard.putCommand("Extender", "Stow", new Stow(arm, gripper, extender));
    SmartShuffleboard.putCommand("Arm", "Manual UP", new ManualMoveArm(arm, 3.0));
    SmartShuffleboard.putCommand("Arm", "Manual DOWN", new ManualMoveArm(arm, -1.5));
    SmartShuffleboard.putCommand("Arm", "ArmMoveSequence 39", new ArmMoveSequence(arm, 39.0));
    SmartShuffleboard.putCommand("Arm", "ArmMoveSequence 28", new ArmMoveSequence(arm, 28.0));
    SmartShuffleboard.putCommand("Arm", "ArmMoveSequence 20", new ArmMoveSequence(arm, 20.0));

    SmartShuffleboard.putCommand("Drive", "Move", new Forward(getDrivetrain()));
    SmartShuffleboard.putCommand("Drive", "ResetGyro", new ResetGyro(getDrivetrain(), 0));
  }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  /*
   public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    /* 
    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());

    Trajectory testTrajectory = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 13.5, new Rotation2d(0)), 
        List.of(new Translation2d(1, 13.8)),
        new Pose2d(2, 13.5, new Rotation2d(0)),
        config);

    drivetrain.getField().getObject("traj").setTrajectory(testTrajectory);
    //change this number to change rotation amount
    double degrees = 90;    
    Supplier<Rotation2d> desiredRot = () -> new Rotation2d(degrees / 180 * Math.PI); 
    
    var thetaController =
      new ProfiledPIDController(
          Constants.kP_THETA, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            testTrajectory,
            drivetrain.getOdometry()::getPoseMeters, // Functional interface to feed supplier
            drivetrain.getKinematics(),
            new PIDController(Constants.kP_X, Constants.kI_X, Constants.kD_X),
            new PIDController(Constants.kP_Y, 0, 0),
            thetaController,
            desiredRot,
            drivetrain::setModuleStates,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(testTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
    
  }*/

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Arm getArm() {
    return arm;
  }
  
  public Extender getExtender() {
    return extender;
  }

}
