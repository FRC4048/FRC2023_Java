
package frc.robot.commands;

import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.utils.ExtenderPosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ExtenderManual extends CommandBase {
    private ExtenderSubsystem extenderSubsystem;
    private int direction; 
    // -1: left, 0: stop, 1: right
  public ExtenderManual(ExtenderSubsystem extenderSubsystem, int direction) {
    addRequirements(extenderSubsystem);
    this.extenderSubsystem = extenderSubsystem;
    this.direction = direction;
  }
  
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    extenderSubsystem.setManualMode(true);
    if(direction == 1){
        extenderSubsystem.setExtenderSpeed(Constants.EXTENDER_SPEED);
    }else if(direction==-1){
        extenderSubsystem.setExtenderSpeed(-Constants.EXTENDER_SPEED);
    }else{
      extenderSubsystem.stopExtender();
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