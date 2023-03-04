// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.drive.StationMoveBack;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StationPickupManual extends SequentialCommandGroup {
  /** Creates a new StationPickupManual. */
  private Drivetrain drivetrain;
  private Extender extender;
  public StationPickupManual(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StationMoveBack(drivetrain),
      new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, 32.0),
             new ParallelRaceGroup(
                 new HoldArmPID(arm, 31.5),
                 new SequentialCommandGroup(
                     new OpenGripper(gripper),
                     new ExtendToPosition(extender, 3550.0), 
                     new CloseGripper(gripper),
                     new ExtendToPosition(extender, 0)
                 )
             ),
             new Stow(arm, gripper, extender)
    );
  }
}
