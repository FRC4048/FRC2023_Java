// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmPositionGrid;
import frc.robot.commands.SetGridSlot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.PieceGrid;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GridAlignment extends SequentialCommandGroup {

  /** Creates a new GridAlignment. */
  public GridAlignment(PhotonCameraSubsystem photonSubsystem, Drivetrain drivetrain, PieceGrid pieceGrid, ArmPositionGrid gridSlot) {
    new AlignAprilTag(photonSubsystem, drivetrain, pieceGrid);
    new WaitCommand(5);//Placeholder
    new SetGridSlot(pieceGrid, gridSlot);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
