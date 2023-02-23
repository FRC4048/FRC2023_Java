// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.drive.Move;
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
public class PickupPiece extends SequentialCommandGroup {
  /** Creates a new PickipPiece. */
  private GripperSubsystem gripper;
  private Arm arm;
  private Extender extender;
  private Drivetrain drivetrain;

  public PickupPiece(GripperSubsystem gripper, Extender extender, Arm arm, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arm = arm;
    this.gripper = gripper;
    this.extender = extender;

    addCommands(
      new Move(drivetrain, -.0508),
      new OpenGripper(gripper),
      new ExtendToPosition(extender, 5700),
      new CloseGripper(gripper),
      new ParallelCommandGroup(new Move(drivetrain, 1),
                               new ExtendToPosition(extender, 0)
      ),
      new SetArmAngle(arm, 0)
      //turn around
    );
  }
}
