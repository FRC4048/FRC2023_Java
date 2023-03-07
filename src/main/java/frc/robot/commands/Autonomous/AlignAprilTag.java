package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;

public class AlignAprilTag extends CommandBase {

  private Pose2d robotFieldPose2d;
  private MoveDistanceSpinTraj moveDistanceSpinTraj;
  private boolean isFinished = false;
  private double startTime;
  private PhotonCameraSubsystem photonSubsystem;
  private Drivetrain drivetrain;

  public AlignAprilTag(PhotonCameraSubsystem photonSubsystem, Drivetrain drivetrain) {
    this.photonSubsystem = photonSubsystem;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    robotFieldPose2d = photonSubsystem.getRobot2dFieldPose();
    if (robotFieldPose2d != null) {
      Double desiredYchangeVertical = robotFieldPose2d.getY();
      Double desiredRotRadians = robotFieldPose2d.getRotation().getRadians();

      if ((desiredYchangeVertical != null) && (desiredRotRadians != null)) {
        moveDistanceSpinTraj = new MoveDistanceSpinTraj(drivetrain, 0.0, -(desiredYchangeVertical), Math.PI);
          moveDistanceSpinTraj.schedule();

        }
      }
      isFinished = true;
    }



  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((System.currentTimeMillis() - startTime) > 5.00) {
      return true;
    }
    return isFinished;
  }
}

