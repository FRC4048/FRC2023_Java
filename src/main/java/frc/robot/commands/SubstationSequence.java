// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmMoveSequence;
import frc.robot.commands.gripper.CloseGripper;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.luxonis.LuxonisVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubstationSequence extends SequentialCommandGroup {
  /** Creates a new SubstationSequence. */
  public SubstationSequence(LuxonisVision luxonisVision, Drivetrain drivetrain, Arm arm, GripperSubsystem gripperSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoSubstation(luxonisVision, drivetrain, 0.6),
      //new ArmMoveSequence(arm, 32.0),    needs extender setpoint added
      //new OpenGripper(gripperSubsystem),   
      new AutoSubstation(luxonisVision, drivetrain, 0),
      new CloseGripper(gripperSubsystem)
    );
  }
}
