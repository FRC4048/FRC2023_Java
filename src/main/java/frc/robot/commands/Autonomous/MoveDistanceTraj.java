package frc.robot.commands.Autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Odometry;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class MoveDistanceTraj extends LoggedCommand {
    
    private Drivetrain drivetrain;
    private Odometry odometry;
    private Pose2d currentPos;
    private Pose2d desiredPos;
    private double xChange;
    private double yChange;
    private TrajectoryConfig config;
    private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kP_THETA_AUTO, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
    private SwerveControllerCommand moveCommand;


    //Command used to move a specific distance without any rotation/angle change
    public MoveDistanceTraj(Drivetrain drivetrain, Odometry odometry, double xChange, double yChange) {
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
        super.initialize();
        //differential drive trajectory generation does not drive in a straight line unless
        //the starting and ending angle are the same, and the starting angle is pointing towards
        //the final point. "Math.atan(yChange/xChange)" creates an angle pointing from currentPos
        //to desiredPos. This angle is ONLY used for generation. Any swerve rotational movement 
        //should be done by passing a rotation2d supplier into the swerveControllerCommand object.
        double angle = Math.atan(yChange/xChange);
        currentPos = new Pose2d(
        odometry.getPoseX(), 
        odometry.getPoseY(), 
        new Rotation2d(angle));

        desiredPos = new Pose2d(
        odometry.getPoseX() + xChange, 
        odometry.getPoseY() + yChange, 
        new Rotation2d(angle));


        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPos, 
        List.of(),
        desiredPos,
        config);

        odometry.getField().getObject("traj").setTrajectory(trajectory);
        moveCommand =
        new SwerveControllerCommand(
            trajectory,                                                                                                                                                                                                                        
            odometry.getOdometry()::getEstimatedPosition, // Functionalp interface to feed supplier
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