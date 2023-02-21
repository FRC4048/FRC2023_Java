// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.List;
import java.util.function.Supplier;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.SetArmAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DepositPickUpEdge extends Autonomous {
  /** Creates a new DepositPickUpLeft. */
  public DepositPickUpEdge(Drivetrain drivetrain, Arm arm, GripperSubsystem gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(drivetrain, arm, gripper);
    //Supplier<Rotation2d> desiredRot = () -> new Rotation2d(Math.PI);

    Trajectory trajectory1 = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(),
        new Pose2d(5.588, .911, new Rotation2d(0)),
        config);

    Trajectory trajectory2 = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.588, .911, new Rotation2d(0)), 
        List.of(),
        new Pose2d(0, 0, new Rotation2d(0)),
        config);
    
    SwerveControllerCommand toGamePiece =
      new SwerveControllerCommand(
          trajectory1,
          drivetrain.getOdometry()::getPoseMeters, // Functional interface to feed supplier
          drivetrain.getKinematics(),
          new PIDController(Constants.kP_X, Constants.kI_X, Constants.kD_X),
          new PIDController(Constants.kP_Y, 0, 0),
          thetaController,
          //desiredRot,
          drivetrain::setModuleStates,
          drivetrain);
  
    SwerveControllerCommand fromGamePiece =
        new SwerveControllerCommand(
            trajectory2,
            drivetrain.getOdometry()::getPoseMeters, // Functional interface to feed supplier
            drivetrain.getKinematics(),
            new PIDController(Constants.kP_X, Constants.kI_X, Constants.kD_X),
            new PIDController(Constants.kP_Y, 0, 0),
            thetaController,
            //desiredRot,
            drivetrain::setModuleStates,
            drivetrain);

    drivetrain.getField().getObject("traj").setTrajectory(trajectory1);
    drivetrain.resetOdometry(trajectory1.getInitialPose());

    addCommands(new WaitCommand(2.5), toGamePiece/*, fromGamePiece*/);
  }
}