/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.logging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Set;
import java.util.TreeSet;

public class LogCommandWrapper extends CommandBase {
  private Command command;
  private String ident;
  private final Set<String> requirements = new TreeSet<String>();

  
  public LogCommandWrapper(Command command) {
    this(command, command.getName());
  }
  
  /**
   * Creates a new LogCommandWrapper.
   */
  public LogCommandWrapper(Command command, String ident) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.command = command;
    this.ident = ident;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log("initialize");
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log("end");
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean result = command.isFinished();
    if(result) {
      log("isFinished");
    }
    return result;
  }
  
  private void log(final String text) {
		final StringBuilder sb = new StringBuilder();
		sb.append(this.getClass().getSimpleName());
		sb.append(" ");
		sb.append(ident);
		// TODO: Cache the string value for requirements to save on creation every time the log message is called
		Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, sb.toString(), requirements.toString(), text);
  }
  
  public Command getWrappedCommand() {
    return command;
  }
}
