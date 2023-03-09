// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.logging.wrappers;

import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.utils.logging.Logging;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ParCommandGroupWrapper extends CommandGroupBase {
  private ParallelCommandGroup parallelCommandGroup;
  private String ident;
  private final Set<String> requirements = new TreeSet<String>();

  /* Creates a new Wrapper. */
  public ParCommandGroupWrapper(ParallelCommandGroup parallelCommandGroup) {
    this(parallelCommandGroup, parallelCommandGroup.getName());
  }

  public ParCommandGroupWrapper(ParallelCommandGroup parallelCommandGroup, String ident) {
    this.parallelCommandGroup = parallelCommandGroup;
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
    parallelCommandGroup.initialize();
  }

  @Override
  public final void execute() {
    parallelCommandGroup.execute();
  }

  @Override
  public final void end(boolean interrupted) {
    parallelCommandGroup.end(interrupted);
  }

  @Override
  public final boolean isFinished() {
    return parallelCommandGroup.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return parallelCommandGroup.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return parallelCommandGroup.getInterruptionBehavior();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    parallelCommandGroup.initSendable(builder);
  }

  @Override
  public void addCommands(Command... commands) {
    parallelCommandGroup.addCommands(commands);
  }
}