// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class BalancePID extends CommandBase {
  /** Creates a new AutoBalance. */
  private Drivetrain drivetrain;
  private double speed;
  private int counter;

  public BalancePID(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    float roll = drivetrain.getFilterRoll();
    float dir = -Math.signum(roll);

    speed = Math.abs(roll) > Constants.BALANCE_THRESH ? MathUtil.clamp(Math.abs((Math.abs(roll) - Constants.BALANCE_THRESH)) * Constants.BALANCE_kP, Constants.BALANCE_LOW_SPEED, Constants.BALANCE_HIGH_SPEED) : 0;

    counter = Math.abs(roll) < Constants.BALANCE_THRESH ? counter+1 : 0;

    drivetrain.drive(dir * speed, 0, 0, true);

    SmartShuffleboard.put("Balance", "PID End", "PID End", counter > Constants.BALANCE_END);
    SmartShuffleboard.put("Balance", "PID Counter", "PID Counter", counter);
    SmartShuffleboard.put("Balance", "PID Speed", "PID Speed", dir * speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > Constants.BALANCE_END;
  }
}
