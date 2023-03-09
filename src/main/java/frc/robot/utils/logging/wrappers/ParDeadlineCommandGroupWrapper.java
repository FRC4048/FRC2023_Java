// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.logging.wrappers;

import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.utils.logging.Logging;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ParDeadlineCommandGroupWrapper extends CommandGroupBase {
  private ParallelDeadlineGroup seqCommandGroup;
  private String ident;
  private final Set<String> requirements = new TreeSet<String>();

  /* Creates a new SequentialCommandGroupWrapper. */
  public ParDeadlineCommandGroupWrapper(ParallelDeadlineGroup seqCommandGroup) {
    this(seqCommandGroup, seqCommandGroup.getName());
  }

  public ParDeadlineCommandGroupWrapper(ParallelDeadlineGroup seqCommandGroup, String ident) {
    this.seqCommandGroup = seqCommandGroup;
    this.ident = ident;
  }

  /* Define how to log events */
  private void log(final String text) {
  final StringBuilder sb = new StringBuilder();
  sb.append(this.getClass().getSimpleName());
  sb.append(" ");
  sb.append(ident);
  Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, sb.toString(), requirements.toString(), text);
  }

  /* Overide events for logging */
  // Called once the command ends or is interrupted.
  @Override
  public void initialize() {
    log("initialize");
    seqCommandGroup.initialize();
  }

  @Override
  public final void execute() {
    seqCommandGroup.execute();
  }

  @Override
  public final void end(boolean interrupted) {
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