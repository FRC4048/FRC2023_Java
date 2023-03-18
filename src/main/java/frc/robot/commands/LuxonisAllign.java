// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.luxonis.LuxonisVision;

public class LuxonisAllign extends CommandBase {
  /** Creates a new LuxonisAllign. */
  LuxonisVision luxonisVision;
  Drivetrain drivetrain;
  int counter;
  double startTime;
  public LuxonisAllign(LuxonisVision luxonisVision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.luxonisVision = luxonisVision;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Math.signum(luxonisVision.getObjectX()) * 0.2;
    boolean targetLock = luxonisVision.hasTargetLock() && (Math.abs(luxonisVision.getObjectX()) < Constants.LUXONIS_THRESH);
    counter = targetLock ? counter + 1 : 0;
    
    SmartShuffleboard.put("Luxonis", "Speed", speed);
    //drivetrain.drive(0, speed, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    SmartShuffleboard.put("Luxonis", "Speed", 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (counter > 6) || ((Timer.getFPGATimestamp() - startTime) > Constants.LUXONIS_ALLIGN_TIMEOUT);
  }
}
