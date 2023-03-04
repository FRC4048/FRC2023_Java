// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class MoveDistanceX extends CommandBase {
  /** Creates a new MoveDistanceX. */
  private Drivetrain drivetrain;
  private double startPos;
  private double driven;
  
  public MoveDistanceX(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = drivetrain.getPoseX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(-0.3, 0.0, 0.0, true);
    //driven = Math.abs(startPos - drivetrain.getPoseX());
    // SmartShuffleboard.put("Substation", "driven", driven);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(startPos - drivetrain.getPoseX()) > 0.63;
  }
}
