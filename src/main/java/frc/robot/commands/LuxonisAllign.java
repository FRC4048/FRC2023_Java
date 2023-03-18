// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.luxonis.LuxonisVision;

public class LuxonisAllign extends CommandBase {
  /** Creates a new LuxonisAllign. */
  LuxonisVision luxonisVision;
  Drivetrain drivetrain;
  public LuxonisAllign(LuxonisVision luxonisVision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.luxonisVision = luxonisVision;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //counter = Math.abs(roll) < Constants.LUXONIS_Y_THRESH ? counter + 1 : 0;

    drivetrain.drive(0, Math.signum(luxonisVision.getObjectY() * 0.2), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    //return (counter > Constants.BALANCE_PID_END) || 
    //       ((Timer.getFPGATimestamp() - startTime) > Constants.CHARGESTATION_TIMEOUT);;
  }
}
