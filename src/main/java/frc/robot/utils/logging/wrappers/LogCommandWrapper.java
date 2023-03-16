/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.logging.wrappers;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.logging.Logging;
import frc.robot.utils.logging.Logging.MessageLevel;

import java.util.Set;
import java.util.TreeSet;

public class LogCommandWrapper extends CommandBase {
  private StringLogEntry initializeEntry;
  private StringLogEntry endEntry;
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
    DataLog log = DataLogManager.getLog();
    this.initializeEntry = new StringLogEntry(log, ident+"-initialize");
    this.endEntry = new StringLogEntry(log, ident+"-end");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initializeEntry.append("Initializing");
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
    this.endEntry.append("Ending");
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
  
  public Command getWrappedCommand() {
    return command;
  }
}
