package frc.robot.commands.drive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveToPosition extends CommandBase {
    
    private Drivetrain drivetrain;
    private Pose2d currentPos;
    private Pose2d desiredPos;
    private double distance;
    private double angle;
    private TrajectoryConfig config;
    private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kP_THETA, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);


    public MoveToPosition(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        config = new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());
        
    }

    @Override
    public void initialize() {
        currentPos = drivetrain.getOdometry().getPoseMeters();
        desiredPos = new Pose2d(currentPos.getX() + 0.5, currentPos.getY(), currentPos.getRotation());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPos, 
        List.of(),
        desiredPos,
        config);

        SwerveControllerCommand moveCommand =
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
          moveCommand.schedule();
    }
    
}