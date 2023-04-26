package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.PieceGrid;
import frc.robot.utils.SmartShuffleboard;

public class AlignAprilTag extends CommandBase {

  private Pose2d robotFieldPose;
  private Pose3d tagPose;
  private PhotonCameraSubsystem photonSubsystem;
  private Drivetrain drivetrain;
  private double desiredYChange;
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
    addRequirements(photonSubsystem, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotFieldPose = photonSubsystem.getRobot2dFieldPose();
    tagPose = photonSubsystem.getTargetFieldPose();
    if (robotFieldPose != null && tagPose != null) {
      desiredYChange = tagPose.getY() - robotFieldPose.getY() + pieceGrid.getSelectedGridSlot().getDistanceFromTagPosition(); // TODO: Check math on this
      new MoveDistanceSpinTraj(drivetrain, 0.0, desiredYChange, Math.PI).schedule();
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
