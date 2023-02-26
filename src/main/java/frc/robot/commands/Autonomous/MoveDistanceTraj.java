package frc.robot.commands.Autonomous;

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
    private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kP_THETA_AUTO, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
    private SwerveControllerCommand moveCommand;


    public MoveDistanceTraj(Drivetrain drivetrain, double xChange, double yChange) {
        this.drivetrain = drivetrain;
        this.xChange = xChange;
        this.yChange = yChange;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        config = new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());
        
    }

    public MoveDistanceTraj(Drivetrain drivetrain, double xChange, double yChange, Rotation2d desiredRot) {
        this.drivetrain = drivetrain;
        this.xChange = xChange;
        this.yChange = yChange;
        config = new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());
        
    }

    @Override
    public void initialize() {
        //differential drive trajectory generation does not drive in a straight line unless
        //the starting and ending angle are the same, and the starting angle is pointing towards
        //the final point. "Math.atan(yChange/xChange)" creates an angle pointing from currentPos
        //to desiredPos. This angle is ONLY used for generation. Any swerve rotational movement 
        //should be done by passing a rotation2d supplier into the swerveControllerCommand object.
        double angle = Math.atan(yChange/xChange);
        currentPos = new Pose2d(
        drivetrain.getPoseX(), 
        drivetrain.getPoseY(), 
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
        moveCommand =
        new SwerveControllerCommand(
            trajectory,                                                                                                                                                                                                                        
            drivetrain.getOdometry()::getPoseMeters, // Functionalp interface to feed supplier
            drivetrain.getKinematics(),
            new PIDController(Constants.kP_X_AUTO, Constants.kI_X_AUTO, Constants.kD_X_AUTO),
            new PIDController(Constants.kP_Y_AUTO, Constants.kI_Y_AUTO, Constants.kD_Y_AUTO),
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);
            moveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return moveCommand.isFinished();
    }
}