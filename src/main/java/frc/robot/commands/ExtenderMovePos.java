// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.utils.ExtenderPosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ExtenderMovePos extends CommandBase {
    private ExtenderSubsystem extenderSubsystem;

    private ExtenderPosition extenderPosition;

  public ExtenderMovePos(ExtenderSubsystem extenderSubsystem,ExtenderPosition extenderPosition) {
    addRequirements(extenderSubsystem);
    this.extenderSubsystem = extenderSubsystem;
    this.extenderPosition = extenderPosition;
  }
  
  @Override
  public void initialize() {
    extenderSubsystem.setManualMode(false);
    extenderSubsystem.extendToPosition(extenderPosition);
  }

  @Override
  public void execute() {

    //extenderSubsystem.setExtenderSpeed(0.5);
    //System.out.println("it works?");
    
  }

  @Override
  public void end(boolean interrupted) {
    extenderSubsystem.stopExtender();
  }

  @Override
  public boolean isFinished() {
    return extenderSubsystem.extendAtPos(extenderPosition); 
  }
}
