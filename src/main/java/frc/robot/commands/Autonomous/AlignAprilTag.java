package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.PieceGrid;

public class AlignAprilTag extends CommandBase {

  private Pose2d robotFieldPose2d;
  private Pose2d tagPose2d;
  private MoveDistanceSpinTraj moveDistanceSpinTraj; //This is needed ignore the warning
  private PhotonCameraSubsystem photonSubsystem;
  private Drivetrain drivetrain;
  PieceGrid pieceGrid;

  /**
   * Aligns the robot to a certain area for drop-off
   * 
   * @param photonSubsystem Where the camera data is being taken from
   * @param drivetrain Drivetrain used for movement
   * @param pieceGrid The area of the grid you want to go to
   */
  public AlignAprilTag(PhotonCameraSubsystem photonSubsystem, Drivetrain drivetrain, PieceGrid pieceGrid) {
    this.photonSubsystem = photonSubsystem;
    this.drivetrain = drivetrain;
    this.pieceGrid = pieceGrid;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotFieldPose2d = photonSubsystem.getRobot2dFieldPose();
    tagPose2d = photonSubsystem.getTargetFieldPose().toPose2d(); // TODO: Check math on this
    if (robotFieldPose2d != null) {
      double desiredYChange = robotFieldPose2d.getY() - tagPose2d.getY() + pieceGrid.getSelectedGridSlot().getDistanceFromTagPosition();
      moveDistanceSpinTraj = new MoveDistanceSpinTraj(drivetrain, 0.0, desiredYChange, Math.PI);
    }
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
    return true;

  }
}
