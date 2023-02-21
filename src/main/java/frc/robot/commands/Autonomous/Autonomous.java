// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  protected Drivetrain drivetrain;
  protected Arm arm;
  protected GripperSubsystem gripper = new GripperSubsystem();

  TrajectoryConfig config = 
    new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());
  ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kP_THETA, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
  
  public Autonomous(Drivetrain drivetrain, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.drivetrain = drivetrain;
    this.arm = arm;
  }
}
