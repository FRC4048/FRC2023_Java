package frc.robot.commands.Autonomous;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.apriltags.AprilTagMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;

public class AlignAprilTag extends CommandBase {

  private Pose2d robotFieldPose2d;
  private MoveDistanceSpinTraj moveDistanceSpinTraj;
  private boolean isFinished = false;
  private double startTime;
  private PhotonCameraSubsystem photonSubsystem;
  private Drivetrain drivetrain;
  private double desiredYchangeVertical;
  private int leftOrRight; // left = 1 and right = 0

  public AlignAprilTag(PhotonCameraSubsystem photonSubsystem, Drivetrain drivetrain, int leftOrRight,
      int desiredYchangeVertical) {
    this.photonSubsystem = photonSubsystem;
    this.drivetrain = drivetrain;
    this.leftOrRight = leftOrRight; // left = 1 and right = 0
    this.desiredYchangeVertical = desiredYchangeVertical;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double moveDistance;
    startTime = System.currentTimeMillis();
    robotFieldPose2d = photonSubsystem.getRobot2dFieldPose();
    if (robotFieldPose2d != null) {
      int currentTagID = photonSubsystem.getTargetId();// gets current Tag Id
      List<AprilTag> aprilTagPosList = AprilTagMap.getAprilPosList(photonSubsystem.getTargetAlliance());// list of Tag pos
      double currPos = robotFieldPose2d.getY(); // current Vertical Pos relative to the field
      double tagPos = aprilTagPosList.get(currentTagID - 1).pose.getY();// gets the vertical positon of the april relative to the field
      boolean isTagGreater = tagPos > currPos ? true : false;// isTagGreater is true if tagPos greater than currPos else false

      if(isTagGreater != false){// if the tag pos is greater
        moveDistance = (leftOrRight == 1) ? (tagPos - desiredYchangeVertical) - currPos : (tagPos + desiredYchangeVertical) - currPos;
        //moveDistance -> if left == true then moveDistance = (tagPos-vertOffset) - currPos else moveDistance = (tagPos + vertOffset) - currPos(subject to change)
      }
      moveDistance = (leftOrRight == 0) ? (tagPos - currPos) - desiredYchangeVertical:(tagPos - currPos) + desiredYchangeVertical;
      //moveDistance -> if right == True then (tagPos - currPos) - vertOffset else (tagPos - currPos) + vertOffset(subject to change)
      double desiredRotRadians = robotFieldPose2d.getRotation().getRadians();
      moveDistanceSpinTraj = new MoveDistanceSpinTraj(drivetrain, 0.0, moveDistance, Math.PI);
      moveDistanceSpinTraj.schedule();

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
