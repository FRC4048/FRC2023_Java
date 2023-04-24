// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.logging.wrappers;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.utils.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ParRaceCommandGroupWrapper extends CommandGroupBase {
  private ParallelRaceGroup ParRaceGroup;
  private String ident;

  /* Creates a new Wrapper. */
  public ParRaceCommandGroupWrapper(ParallelRaceGroup ParRaceGroup) {
    this(ParRaceGroup, ParRaceGroup.getName());
  }

  public ParRaceCommandGroupWrapper(ParallelRaceGroup ParRaceGroup, String ident) {
    this.ParRaceGroup = ParRaceGroup;
    this.ident = ident;
    m_requirements.addAll(ParRaceGroup.getRequirements());
  }
  
  /* Overide events for logging */
  // Called once the command ends or is interrupted.
  @Override
  public void initialize() {
    Logger.logBoolean("/commands/" + ident, true, Constants.ENABLE_LOGGING);
    ParRaceGroup.initialize();
  }

  @Override
  public final void execute() {
    ParRaceGroup.execute();
  }

  @Override
  public final void end(boolean interrupted) {
    Logger.logBoolean("/commands/" + ident, false, Constants.ENABLE_LOGGING);
    ParRaceGroup.end(interrupted);
  }

  @Override
  public final boolean isFinished() {
    return ParRaceGroup.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return ParRaceGroup.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return ParRaceGroup.getInterruptionBehavior();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    ParRaceGroup.initSendable(builder);
  }

  @Override
  public void addCommands(Command... commands) {
    ParRaceGroup.addCommands(commands);
  }
}