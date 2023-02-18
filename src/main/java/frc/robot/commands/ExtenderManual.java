
package frc.robot.commands;

import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.utils.ExtenderPosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ExtenderManual extends CommandBase {
    private ExtenderSubsystem extenderSubsystem;
    private boolean direction;
  public ExtenderManual(ExtenderSubsystem extenderSubsystem, boolean direction) {
    addRequirements(extenderSubsystem);
    this.extenderSubsystem = extenderSubsystem;
    this.direction =direction;
  }
  
  @Override
  public void initialize() {
     
  }

  @Override
  public void execute() {
    extenderSubsystem.setExtenderSpeed(Constants.EXTENDER_SPEED);
    if(direction){
      extenderSubsystem.extendToPositionManual(500);
    }else{
      extenderSubsystem.extendToPositionManual(-500);
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