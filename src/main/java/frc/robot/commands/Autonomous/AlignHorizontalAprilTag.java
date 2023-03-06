// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;



public class AlignHorizontalAprilTag extends CommandBase {


  private Pose2d robotFieldPose2d;
  private MoveDistanceTraj moveDistanceTraj;
  private boolean isFinished = false;
  private double startTime;

  
  public AlignHorizontalAprilTag(PhotonCameraSubsystem photonSubsystem, Drivetrain drivetrain) {

    robotFieldPose2d = photonSubsystem.getRobot2dFieldPose();
    double desiredXchangeHorizontal = robotFieldPose2d.getTranslation().getX();
    moveDistanceTraj = new MoveDistanceTraj(drivetrain, -(desiredXchangeHorizontal), 0.0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    if (!moveDistanceTraj.isScheduled()) {
      moveDistanceTraj.schedule();
    }
    
    
    isFinished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((System.currentTimeMillis() - startTime) > 5.00 ) {
      return true;
    }
    return isFinished;
  }
}
