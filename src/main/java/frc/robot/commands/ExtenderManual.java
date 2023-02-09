
package frc.robot.commands;

import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.utils.ExtenderPosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

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
    extenderSubsystem.setManualMode(true);

  }

  @Override
  public void execute() {
    extenderSubsystem.setManualMode(true);
    if(direction == true){
        extenderSubsystem.setExtenderSpeed(Constants.EXTENDER_SPEED);
    }else{
        extenderSubsystem.setExtenderSpeed(-Constants.EXTENDER_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    extenderSubsystem.stopExtender();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}