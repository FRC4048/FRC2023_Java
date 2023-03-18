// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class CrossPanel extends CommandBase {
  /** Creates a new DriveOverPanel. */
  private Drivetrain drivetrain;
  private boolean climbing;
  private boolean crossed;
  private double startTime;
  private int counter;

  public CrossPanel(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbing = false;
    crossed = false;
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter = !climbing ? (Math.abs(drivetrain.getFilterRoll()) > 5 ? counter + 1 : 0) : (Math.abs(drivetrain.getFilterRoll()) < 2 ? counter + 1 : 0);

    climbing = !climbing ? (counter > Constants.CROSS_CLIMB) : climbing;

    crossed = (counter > Constants.CROSS_END) && climbing;

    drivetrain.drive(.6, 0, 0, true);

    SmartShuffleboard.put("Cross", "Counter", "Counter", counter);
    SmartShuffleboard.put("Cross", "Climbing", "Climbing", climbing);
    SmartShuffleboard.put("Cross", "Crossed", "Crossed", crossed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    SmartShuffleboard.put("Cross", "Counter", "Counter", counter);
    SmartShuffleboard.put("Cross", "Climbing", "Climbing", climbing);
    SmartShuffleboard.put("Cross", "Crossed", "Crossed", crossed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return crossed || ((Timer.getFPGATimestamp() - startTime) > Constants.CHARGESTATION_TIMEOUT);
  }
}
