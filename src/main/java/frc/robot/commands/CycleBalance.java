// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class CycleBalance extends LoggedCommand {
  /** Creates a new CycleBalance. */
  private Drivetrain drivetrain;
  private int climbCounter;
  private int finishCounter;
  private boolean climbing;

  public CycleBalance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    climbCounter = 0;
    finishCounter = 0;
    climbing = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(Constants.BALANCE_HIGH_SPEED, 0, 0, true);

    climbCounter = (drivetrain.getFilterRoll() > 10 && drivetrain.getFilterRoll() < 15) && !climbing  ? climbCounter + 1 : 0;
    climbing = climbCounter > 10;

    finishCounter = drivetrain.getFilterRoll() < 9 && climbing ? finishCounter + 1 : 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishCounter > Constants.BALANCE_PID_END;
  }
}
