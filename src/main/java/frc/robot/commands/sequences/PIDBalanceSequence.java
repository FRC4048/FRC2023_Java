// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalancePID;
import frc.robot.commands.BalanceSteep;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.StopDriveTrain;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDBalanceSequence extends SequentialCommandGroup {
  /** Creates a new PIDBalanceSequence. */
  public PIDBalanceSequence(Drivetrain drivetrain, boolean forward) {
    setName("PIDBalanceSequence");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BalanceSteep(drivetrain, forward),
      new BalancePID(drivetrain),
      new LockWheels(drivetrain),
      new StopDriveTrain(drivetrain)
    );
  }
}
