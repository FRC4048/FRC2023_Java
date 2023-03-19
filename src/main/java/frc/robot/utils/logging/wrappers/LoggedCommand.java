/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.logging.wrappers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;

public class LoggedCommand extends CommandBase {

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.logBoolean("/Commands/" + getName(), true, Constants.ENABLE_LOGGING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.logBoolean("/Commands/" + getName(), false, Constants.ENABLE_LOGGING);
  }
}
