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
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StationPickupManual extends SequentialCommandGroup {
  /** Creates a new StationPickupManual. */
  public StationPickupManual(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper) {
    setName("StationPickupManualSequence");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StationMoveBack(drivetrain, -0.3),
      new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, Constants.SUBSTATION_PICKUP_ANGLE),
      new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
                 new HoldArmPID(arm, Constants.SUBSTATION_PICKUP_ANGLE),
                 new SequentialCommandGroupWrapper(new SequentialCommandGroup(
                     new OpenGripper(gripper),
                     new ExtendToPosition(extender, Constants.SUBSTATION_PICKUP_EXTENSION) 
                     // new CloseGripper(gripper)
                 ), "StationPickupManualPositionSequence"
             )
             // new Stow(arm, gripper, extender)
    ), "StationPickupManualParRace"));
  }
}
