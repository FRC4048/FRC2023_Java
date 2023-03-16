// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.logging.wrappers;

import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.logging.Logging;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialCommandGroupWrapper extends CommandGroupBase {
  private StringLogEntry initializeEntry;
  private StringLogEntry endEntry;
  private SequentialCommandGroup seqCommandGroup;
  private String ident;
  private final Set<String> requirements = new TreeSet<String>();

  /* Creates a new SequentialCommandGroupWrapper. */
  public SequentialCommandGroupWrapper(SequentialCommandGroup seqCommandGroup) {
    this(seqCommandGroup, seqCommandGroup.getName());
  }

  public SequentialCommandGroupWrapper(SequentialCommandGroup seqCommandGroup, String ident) {
    this.seqCommandGroup = seqCommandGroup;
    this.ident = ident;
    DataLog log = DataLogManager.getLog();
    this.initializeEntry = new StringLogEntry(log, ident+ "-initialize");
    this.endEntry = new StringLogEntry(log, ident+"-end");
  }

  /* Overide events for logging */
  // Called once the command ends or is interrupted.
  @Override
  public void initialize() {
    this.initializeEntry.append("Initializing");
    seqCommandGroup.initialize();
  }

  @Override
  public final void execute() {
    seqCommandGroup.execute();
  }

  @Override
  public final void end(boolean interrupted) {
    this.initializeEntry.append("Ending");
    seqCommandGroup.end(interrupted);
  }

  @Override
  public final boolean isFinished() {
    return seqCommandGroup.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return seqCommandGroup.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return seqCommandGroup.getInterruptionBehavior();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    seqCommandGroup.initSendable(builder);
  }

  @Override
  public void addCommands(Command... commands) {
    seqCommandGroup.addCommands(commands);
  }
}