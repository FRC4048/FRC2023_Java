// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Autonomous.MoveDistanceSpinTraj;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.luxonis.LuxonisVision;

public class SubstationTrajAllign extends CommandBase {
  /** Creates a new SubstationTrajAllign. */
  Drivetrain drivetrain;
  LuxonisVision luxonisVision;
  double offset;
  public SubstationTrajAllign(Drivetrain drivetrain, LuxonisVision luxonisVision, double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.luxonisVision = luxonisVision;
    this.offset = offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new MoveDistanceSpinTraj(drivetrain, luxonisVision.getObjectZ()-offset, -luxonisVision.getObjectX(), 0).schedule();;
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
    return true;
  }
}
