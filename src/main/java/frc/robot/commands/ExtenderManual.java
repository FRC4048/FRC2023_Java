
package frc.robot.commands;

import frc.robot.subsystems.ExtenderSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ExtenderManual extends CommandBase {
    private ExtenderSubsystem extenderSubsystem;
    private boolean direction;
    
  public ExtenderManual(ExtenderSubsystem extenderSubsystem, boolean direction) {
    addRequirements(extenderSubsystem);
    this.extenderSubsystem = extenderSubsystem;
    this.direction = direction;
  }
  
  @Override
  public void initialize() {
     
  }

  @Override
  public void execute() {
    extenderSubsystem.setExtenderSpeed(Constants.EXTENDER_SPEED);
    if(direction){
      extenderSubsystem.extendToPositionManual(Constants.EXTENDER_ENCODER_MANUAL);
    }else{
      extenderSubsystem.extendToPositionManual(-Constants.EXTENDER_ENCODER_MANUAL);
    }
  }

  @Override
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}