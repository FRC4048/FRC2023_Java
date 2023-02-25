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

public class AutoSubstationAllign extends CommandBase {
  /** Creates a new AutoSubstationAllign. */
  LuxonisVision luxonisVision;
  Drivetrain drivetrain;
  double startTime;
  double speed;
  double dist;
  double initPos;
  boolean foundTarget;
  public AutoSubstationAllign(LuxonisVision luxonisVision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.luxonisVision = luxonisVision;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    dist = luxonisVision.getObjectX();
    initPos = drivetrain.getPoseX();
    foundTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = 0;
    if (!foundTarget) {
      dist = luxonisVision.getObjectX();
    }
    if (!foundTarget & dist != 0) {
        foundTarget = true;
    }


    if (foundTarget) {
      drivetrain.drive(0, -0.01*Math.signum(dist), 0,  true);
    }
    SmartShuffleboard.put("Substation","Dist", dist);
    SmartShuffleboard.put("Substation","Target", foundTarget);
    SmartShuffleboard.put("Substation","Obj", luxonisVision.getObjectX());
    SmartShuffleboard.put("Substation","Dist", drivetrain.getPoseX());
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
    return (foundTarget & drivetrain.getPoseX() < initPos-dist*0.8 & drivetrain.getPoseX() > initPos-dist*1.2) || Timer.getFPGATimestamp()-startTime > 5;
    // (luxonisVision.getObjectX() < 0.03 & luxonisVision.getObjectX() > -0.03) ||
  }

}
