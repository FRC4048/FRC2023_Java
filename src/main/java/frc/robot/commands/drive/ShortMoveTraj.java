package frc.robot.commands.drive;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ShortMoveTraj {

    private TrajectoryConfig config;
    private Drivetrain drivetrain;

    ProfiledPIDController thetaController = 
    new ProfiledPIDController(Constants.kP_THETA, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
    
    public ShortMoveTraj(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        config = new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO)
        .setKinematics(drivetrain.getKinematics());
    }

    Trajectory trajectory1 = 
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(),
      new Pose2d(0.5, 0, new Rotation2d(0)),
      config);

}
