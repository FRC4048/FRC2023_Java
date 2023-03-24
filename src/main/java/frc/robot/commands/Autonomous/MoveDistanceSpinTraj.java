package frc.robot.commands.Autonomous;

import java.util.List;
import java.util.function.Supplier;

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
import frc.robot.utils.logging.wrappers.LoggedCommand;


public class MoveDistanceSpinTraj extends LoggedCommand {
    
    private Drivetrain drivetrain;
    private Pose2d currentPos;
    private Pose2d desiredPos;
    private double xChange;
    private double yChange;
    private double desiredRotRadians;
    private Supplier<Rotation2d> desiredRotSupplier;
    private TrajectoryConfig config;
    private ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kP_THETA_AUTO, Constants.kI_THETA_AUTO, Constants.kD_THETA_AUTO, Constants.THETA_CONTROLLER_CONSTRAINTS);
    private SwerveControllerCommand moveCommand;

    //Command used to move a specific distance and turn to a specific angle
    public MoveDistanceSpinTraj(
        Drivetrain drivetrain, 
        double xChange, 
        double yChange, 
        double desiredRotRadians) {
        this.drivetrain = drivetrain;
        this.xChange = xChange;
        this.yChange = yChange;
        this.desiredRotRadians = desiredRotRadians;
        config = new TrajectoryConfig(Constants.MAX_VELOCITY_AUTO, Constants.MAX_ACCELERATION_AUTO).setKinematics(drivetrain.getKinematics());
        desiredRotSupplier = () -> new Rotation2d(desiredRotRadians);

        }

    @Override
    public void initialize() {
        super.initialize();
        //differential drive trajectory generation does not drive in a straight line unless
        //the starting and ending angle are the same, and the starting angle is pointing towards
        //the final point. "Math.atan(yChange/xChange)" creates an angle pointing from currentPos
        //to desiredPos. This angle is ONLY used for generation. Any swerve rotational movement 
        //should be done by passing a rotation2d supplier into the swerveControllerCommand object.
        double angle = getTargetAngle();
        currentPos = new Pose2d(
        drivetrain.getPoseX(), 
        drivetrain.getPoseY(), 
        new Rotation2d(angle));

        desiredPos = new Pose2d(
        drivetrain.getPoseX() + xChange, 
        drivetrain.getPoseY() + yChange, 
        new Rotation2d(angle));


        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPos, 
        List.of(),
        desiredPos,
        config);

        Logger.logTrajectory("/movedistancetraj", trajectory, Constants.ENABLE_LOGGING);

        drivetrain.getField().getObject("traj").setTrajectory(trajectory);

        moveCommand =
        new SwerveControllerCommand(
          trajectory,                                                                                                                                                                                                                        
          drivetrain.getOdometry()::getEstimatedPosition, // Functionalp interface to feed supplier
          drivetrain.getKinematics(),
          new PIDController(Constants.kP_X_AUTO, Constants.kI_X_AUTO, Constants.kD_X_AUTO),
          new PIDController(Constants.kP_Y_AUTO, Constants.kI_Y_AUTO, Constants.kD_Y_AUTO),
          thetaController,
          desiredRotSupplier,
          drivetrain::setModuleStates,
          drivetrain
          );
          moveCommand.schedule();
    }

    // The calculated angle must be the absolute angle on a unit circle otherwise the
    // pose estimator might rotate the robot during the move. Calculate the direction of the 
    // vector of (Y2 - Y2) / (X2 - X1) by adding the necessary quadrant adjustments to the atan.
    private double getTargetAngle() {
        double angle = 0;
        if (xChange != 0) { // very important, otherwise divide by 0 errors
            double arctan = Math.abs(Math.atan((yChange) / xChange));
            if (xChange > 0 && yChange >= 0) {
                // first quadrant
                angle = arctan;
            } else if (xChange < 0 && yChange >= 0) {
                // second quadrant
                angle = Math.PI - arctan;
            } else if (xChange < 0 && yChange < 0) {
                // third quadrant
                angle = Math.PI + arctan;
            } else {
                angle = (Math.PI*2) - arctan;
            }
        } else {
            // Trick the trajectory generator to think the robot is facing up or down
            // This is only when the desired X change is exactly 0
            angle = (Math.PI / 2) * Math.signum(yChange);
        }
        return angle;
    }

    public SwerveControllerCommand getMoveCommand(){
        return moveCommand;
    }

    @Override
    public boolean isFinished() {
        return moveCommand.isFinished();
    }
}