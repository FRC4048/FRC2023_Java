// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class AutoCloseGripper extends CommandBase {
  /** Creates a new AutoCloseGripper. */
  private Arm arm;
  private GripperSubsystem gripper;
  private boolean overSubstation;
  private double initTime;
  private double cycleCounter;
  public AutoCloseGripper(Arm arm, GripperSubsystem gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.gripper = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    overSubstation = false;
    initTime = Timer.getFPGATimestamp();
    cycleCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getDistance() < Constants.AUTO_CLOSE_GRIP_DISTANCE && arm.getDistance() > 0) {
      cycleCounter++;
    } else {
      cycleCounter = 0;
    }

    if (cycleCounter > Constants.AUTO_CLOSE_GRIP_CYCLES) {
      overSubstation = true;
    }

    if (Constants.ARM_DEBUG) {
    SmartShuffleboard.put("Arm", "Gripper Close", overSubstation);
    SmartShuffleboard.put("Arm", "Auto Gripper Time Left", Constants.AUTO_CLOSE_GRIPPER_TIMEOUT - (Timer.getFPGATimestamp() - initTime));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - initTime > Constants.AUTO_CLOSE_GRIPPER_TIMEOUT) || overSubstation;
  }
}
