// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmMoveSequence;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubstationPickupSequence extends SequentialCommandGroup {
  /** Creates a new PickipPiece. */
  private GripperSubsystem gripper;
  private Arm arm;
  private Extender extender;
  private Drivetrain drivetrain;

  public SubstationPickupSequence(GripperSubsystem gripper, Extender extender, Arm arm, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arm = arm;
    this.gripper = gripper;
    this.extender = extender;

    addCommands(
      //align with luxonis
      new ArmMoveSequence(arm, Constants.SUBSTATION_PICKUP_ARM_ANGLE),
      new ExtendToPosition(extender, 10), //THIS NUMBER NEEDS TO BE BASED OFF OF LUXONIS DATA
      new CloseGripper(gripper),
      new ExtendToPosition(extender, 0),
      new VoltageMoveArm(arm, Constants.ARM_STOW_SPEED, 0.0)
      //turn around
    );
  }
}
