// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Logger;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class BalanceSteep extends LoggedCommand {
  /** Creates a new BalanceSteep. */
  private Drivetrain drivetrain;
  private int counter;
  private boolean forward;

  public BalanceSteep(Drivetrain drivetrain, boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.forward = forward;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = forward ? Constants.BALANCE_STEEP_SPEED : -Constants.BALANCE_STEEP_SPEED;

    Logger.logDouble("/filterRoll", (double) drivetrain.getFilterRoll(), Constants.ENABLE_LOGGING);

    drivetrain.drive(speed, 0, 0, true);
    
    if (Constants.DRIVETRAIN_DEBUG) {
      SmartShuffleboard.put("Balance", "Steep Speed", "Steep Speed", speed);
      SmartShuffleboard.put("Balance", "Steep End", "Steep End", counter > Constants.BALANCE_STEEP_END);
      SmartShuffleboard.put("Balance", "Steep Counter", "Steep Counter", counter);
    }

    counter = Math.abs(drivetrain.getFilterRoll()) > Constants.BALANCE_STEEP ? counter + 1 : 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.stopMotors();
    SmartShuffleboard.put("Balance", "Steep End", "Steep End", counter > Constants.BALANCE_STEEP_END);
    SmartShuffleboard.put("Balance", "Steep Counter", "Steep Counter", counter);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > Constants.BALANCE_STEEP_END;
  }
}
