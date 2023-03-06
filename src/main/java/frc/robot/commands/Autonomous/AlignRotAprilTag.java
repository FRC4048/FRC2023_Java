package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;

public class AlignRotAprilTag extends CommandBase {

  private Pose2d robotFieldPose2d;
  private PhotonCameraSubsystem photonSubsystem;
  // private double startTime;
  private Drivetrain drivetrain;
  private MoveDistanceSpinTraj moveDistanceSpinTraj;
  private boolean isFinished = false;
  private double startTime;

  public AlignRotAprilTag(PhotonCameraSubsystem photonSubsystem, Drivetrain drivetrain) {
    this.photonSubsystem = photonSubsystem;
    this.drivetrain = drivetrain;
    robotFieldPose2d = photonSubsystem.getRobot2dFieldPose();
    double desiredRotRadians = robotFieldPose2d.getRotation().getRadians();

    moveDistanceSpinTraj = new MoveDistanceSpinTraj(drivetrain, 0.0, 0.0, -(desiredRotRadians));
 
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    moveDistanceSpinTraj.schedule();
    isFinished = true;
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if ((System.currentTimeMillis() - startTime) > 5.00 ) {
      return true;
    }
    return isFinished;
  }
}
