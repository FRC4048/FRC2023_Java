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
    private double initTime;
    private ExtenderPosition extenderPosition;

  public ExtenderMovePos(ExtenderSubsystem extenderSubsystem,ExtenderPosition extenderPosition) {
    addRequirements(extenderSubsystem);
    this.extenderSubsystem = extenderSubsystem;
    this.extenderPosition = extenderPosition;
  }
  
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    extenderSubsystem.setManualMode(false);
    extenderSubsystem.extendToPosition(extenderPosition);
  }

  @Override
  public void execute() {
    
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

 /* 
 @Override

    public enum ExtenderDirection {
        EXTENDFULL, RETRACTFULL
    }    

     private ExtenderDirection direction;


    public ExtenderCommand(ExtenderSubsystem extenderSubsystem, ExtenderDirection direction) {
        addRequirements(extenderSubsystem);
        this.extenderSubsystem = extenderSubsystem;
        this.direction = direction;
    }

    execute:
    double speed = 0;
    if(direction==ExtenderDirection.EXTENDFULL && !extenderSubsystem.getBottomSwitch()){
        speed = -Constants.EXTENDER_SPEED;
    }else if(direction ==ExtenderDirection.RETRACTFULL &&!extenderSubsystem.getTopSwitch()){
        speed = Constants.EXTENDER_SPEED;
    }
    extenderSubsystem.setEntenderSpeed(speed);

  
    public boolean isFinished() {
    return
        (((extenderSubsystem.getTopSwitch() && (direction == ExtenderDirection.EXTENDFULL)) 
        ||
        (extenderSubsystem.getBottomSwitch() && (direction == ExtenderDirection.RETRACTFULL)) 
        ||
        ((Timer.getFPGATimestamp() - initTime) >= Constants.EXTENDER_MOTOR_TIMEOUT)));
  }
}

  */
 
