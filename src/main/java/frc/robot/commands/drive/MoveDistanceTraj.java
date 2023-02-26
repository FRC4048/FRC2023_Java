package frc.robot.commands.drive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveDistanceTraj extends CommandBase {
    
    private Drivetrain drivetrain;
    private Pose2d currentPos;
    private Pose2d desiredPos;
    private double xChange;
    private double yChange;
    private TrajectoryConfig config;
    private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kP_THETA, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);


    public MoveDistanceTraj(Drivetrain drivetrain, double xChange, double yChange) {
        this.drivetrain = drivetrain;
        this.xChange = xChange;
        this.yChange = yChange;
        config = new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());
        
    }

    @Override
    public void initialize() {
        double angle = Math.atan(yChange/xChange);
        currentPos = new Pose2d(
        drivetrain.getOdometry().getPoseMeters().getX(), 
        drivetrain.getOdometry().getPoseMeters().getY(), 
        new Rotation2d(angle));

        desiredPos = new Pose2d(
        drivetrain.getOdometry().getPoseMeters().getX() + xChange, 
        drivetrain.getOdometry().getPoseMeters().getY() + yChange, 
        new Rotation2d(angle));


        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPos, 
        List.of(),
        desiredPos,
        config);

        drivetrain.getField().getObject("traj").setTrajectory(trajectory);

        SwerveControllerCommand moveCommand =
      new SwerveControllerCommand(
          trajectory,                                                                                                                                                                                                                        
          drivetrain.getOdometry()::getPoseMeters, // Functionalp interface to feed supplier
          drivetrain.getKinematics(),
          new PIDController(Constants.kP_X, Constants.kI_X, Constants.kD_X),
          new PIDController(Constants.kP_Y, Constants.kI_Y, Constants.kD_Y),
          thetaController,
          //desiredRot,
          drivetrain::setModuleStates,
          drivetrain);
          moveCommand.schedule();
    }
}