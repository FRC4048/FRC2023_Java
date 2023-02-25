// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.luxonis.LuxonisVision;

public class AutoSubstation extends CommandBase {
  /** Creates a new AutoSubstationAllign. */
  LuxonisVision luxonisVision;
  Drivetrain drivetrain;
  double startTime;
  double speedX, speedY;
  double xOffset, zOffset;
  double initXPos, initYPos;
  boolean foundTarget;
  double distanceToStop;
  public AutoSubstation(LuxonisVision luxonisVision, Drivetrain drivetrain, double distanceToStop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.luxonisVision = luxonisVision;
    this.drivetrain = drivetrain;
    this.distanceToStop = distanceToStop;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    xOffset = luxonisVision.getObjectX();
    zOffset = luxonisVision.getObjectZ();
    initXPos = drivetrain.getPoseX();
    initYPos = drivetrain.getPoseY();

    foundTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedX = 0;
    speedY = 0;
    if (zOffset != 0) {
      zOffset -= distanceToStop;
    }
    if (!(Math.abs(xOffset*0.9) < Math.abs(initYPos-drivetrain.getPoseY()) & Math.abs(xOffset*1.1) > Math.abs(initYPos-drivetrain.getPoseY()))) {
      speedY = 0.01*Math.signum(xOffset);
    }

    if (!(Math.abs(zOffset*0.9) < Math.abs(initXPos-drivetrain.getPoseX()) & Math.abs(zOffset*1.1) > Math.abs(initXPos-drivetrain.getPoseX()))) {
      speedX = 0.01*Math.signum(zOffset);
    }

    //if (!foundTarget) {
      //dist = luxonisVision.getObjectX();    
    //}
    //if (!foundTarget & dist != 0) {
      //  foundTarget = true;
    //}


    if (foundTarget) {
      drivetrain.drive(-speedX, -speedY, 0,  true);
    }
    SmartShuffleboard.put("Substation","Delta Offset", xOffset);
    SmartShuffleboard.put("Substation","Target", foundTarget);
    SmartShuffleboard.put("Substation","Obj", luxonisVision.getObjectX());
    SmartShuffleboard.put("Substation","Robot Location", drivetrain.getPoseY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Timer.getFPGATimestamp()-startTime > 5;
    return (speedX == 0 & speedY == 0) || Timer.getFPGATimestamp()-startTime > 5;
    // (luxonisVision.getObjectX() < 0.03 & luxonisVision.getObjectX() > -0.03) ||
  }

}
