package frc.robot.commands.drive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.Autonomous.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class MoveToPosition extends Autonomous {
    
    private Drivetrain drivetrain;
    private Pose2d currentPos;
    private Pose2d desiredPos;
    private double distance;
    private double angle;

    public MoveToPosition(Drivetrain drivetrain) {
        super(drivetrain);
        currentPos = new Pose2d(0, 0, new Rotation2d(0));
        desiredPos = new Pose2d(0.25, 0, new Rotation2d(0));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPos, 
        List.of(),
        desiredPos,
        config);

        SwerveControllerCommand toGamePiece =
      new SwerveControllerCommand(
          trajectory,
          drivetrain.getOdometry()::getPoseMeters, // Functional interface to feed supplier
          drivetrain.getKinematics(),
          new PIDController(Constants.kP_X, Constants.kI_X, Constants.kD_X),
          new PIDController(Constants.kP_Y, 0, 0),
          thetaController,
          //desiredRot,
          drivetrain::setModuleStates,
          drivetrain);
    }
    
}