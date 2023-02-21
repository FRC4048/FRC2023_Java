// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.Drive;
import frc.robot.commands.GyroOffseter;
import frc.robot.commands.ArmController;
import frc.robot.commands.ManualMoveGripper;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.Autonomous.DepositPickUpEdge;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PowerDistributionBoard;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.XboxController;
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
  private GripperSubsystem gripper;
  private Joystick joyLeft = new Joystick(Constants.LEFT_JOYSICK_ID);
  private Joystick joyRight = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  private JoystickButton LeftGyroButton= new JoystickButton(joyLeft, 1);
  private JoystickButton RightGyroButton= new JoystickButton(joyRight, 1);
  private JoystickButton button_1 = new JoystickButton(joyLeft, 2);
  private JoystickButton button_3 = new JoystickButton(joyLeft, 3);
  private XboxController xbox = new XboxController(2);
  private CommandXboxController cmdController = new CommandXboxController(2);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    drivetrain = new Drivetrain();
    gripper = new GripperSubsystem();
    arm = new Arm();
    m_PDB = new PowerDistributionBoard();

    configureBindings();
    drivetrain.setDefaultCommand(new Drive(drivetrain, () -> joyLeft.getY(), () -> joyLeft.getX(), ()-> joyRight.getX()));
    gripper.setDefaultCommand(new ManualMoveGripper(gripper, () -> xbox.getLeftY()));
  }

  private void configureBindings() {
    LeftGyroButton.onTrue(new GyroOffseter(drivetrain, -1));
    RightGyroButton.onTrue(new GyroOffseter(drivetrain, +1));
    button_1.onTrue(new CloseGripper(gripper));
    button_3.onTrue(new OpenGripper(gripper));
    cmdController.rightBumper().whileTrue(new ArmController(arm, Constants.ARM_CONTROLLER_CHANGE));
    cmdController.leftBumper().whileTrue(new ArmController(arm, -1 * Constants.ARM_CONTROLLER_CHANGE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  
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
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false)); */
    return new DepositPickUpEdge(drivetrain, arm).andThen(() -> drivetrain.drive(0,0,0, false));
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Arm getArm() {
    return arm;
  }
}
