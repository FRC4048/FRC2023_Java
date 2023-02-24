// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.luxonis.LuxonisVision;

public class AutoSubstationAllign extends CommandBase {
  /** Creates a new AutoSubstationAllign. */
  LuxonisVision luxonisVision;
  Drivetrain drivetrain;
  double startTime;
  double speed;
  public AutoSubstationAllign(LuxonisVision luxonisVision, Drivetrain drivetrain) {
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
    speed = Math.signum(luxonisVision.getObjectX()) * 0.1;
    drivetrain.drive(0, speed, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp()-startTime > 5;
  }
}
