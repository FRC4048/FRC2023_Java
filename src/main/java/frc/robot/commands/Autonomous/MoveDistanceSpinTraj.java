package frc.robot.commands.Autonomous;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
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

public class MoveDistanceSpinTraj extends CommandBase {
    
    private Drivetrain drivetrain;
    private Pose2d currentPos;
    private Pose2d desiredPos;
    private double xChange;
    private double yChange;
    private double desiredRotRadians;
    private Supplier<Rotation2d> desiredRotSupplier;
    private TrajectoryConfig config;
    private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kP_THETA, Constants.kI_THETA, Constants.kD_THETA, Constants.THETA_CONTROLLER_CONSTRAINTS);


    public MoveDistanceSpinTraj(Drivetrain drivetrain, double xChange, double yChange, double desiredRotRadians) {
        this.drivetrain = drivetrain;
        this.xChange = xChange;
        this.yChange = yChange;
        this.desiredRotRadians = desiredRotRadians;
        config = new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());
        desiredRotSupplier = () -> new Rotation2d(desiredRotRadians);

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
          desiredRotSupplier,
          drivetrain::setModuleStates,
          drivetrain
          );
          moveCommand.schedule();
    }
}