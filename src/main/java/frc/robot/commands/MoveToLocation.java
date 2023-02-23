// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveToLocation extends CommandBase {
  /** Creates a new MoveToLocation. */
  Drivetrain drivetrain;
  double x_distance;
  double speed;
  public MoveToLocation(Drivetrain drivetrain, double x_distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.x_distance = x_distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = Constants.AUTO_CHARGESTATION_SPEED * drivetrain.getFilterAccelX()/9.8 * Math.signum(x_distance-drivetrain.getPoseX());
    drivetrain.drive(speed, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drivetrain.getPoseX() > x_distance*1.1 & drivetrain.getPoseX() < x_distance*0.9);
  }
}
