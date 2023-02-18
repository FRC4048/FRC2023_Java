// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.utils.ExtenderPosition;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ExtenderMovePos extends CommandBase {
    private ExtenderSubsystem extenderSubsystem;

    private ExtenderPosition extenderPosition;
    private double initTime = Timer.getFPGATimestamp(); 

  public ExtenderMovePos(ExtenderSubsystem extenderSubsystem,ExtenderPosition extenderPosition) {
    addRequirements(extenderSubsystem);
    this.extenderSubsystem = extenderSubsystem;
    this.extenderPosition = extenderPosition;
  }
  
  @Override
  public void initialize() {
    extenderSubsystem.resetEncoder();
    extenderSubsystem.setManualMode(false);
    extenderSubsystem.stopExtender();
  }

  @Override
  public void execute() {
  
  extenderSubsystem.setExtenderSpeed(Constants.EXTENDER_SPEED);
    
  }

  @Override
  public void end(boolean interrupted) {


    extenderSubsystem.setExtenderSpeed(0);
  }

  @Override
  public boolean isFinished() {

  //  if ((Timer.getFPGATimestamp() - initTime) > 5000 ) {
  //    return true;
  //  }
    return extenderSubsystem.extendAtPos(extenderPosition); 
    
  }
}
