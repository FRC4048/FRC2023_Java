// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.commands.SetGridSlot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.PieceGrid;
import frc.robot.utils.SmartShuffleboard;

public class GridAlignment extends SequentialCommandGroup {

  public GridAlignment(PhotonCameraSubsystem photonSubsystem, Drivetrain drivetrain, PieceGrid pieceGrid, ArmPositionGrid gridSlot) {
    setName("DepositOneAndBalanceSequence");
    addCommands(
            new AlignAprilTag(photonSubsystem, drivetrain, pieceGrid),
            new SetGridSlot(pieceGrid, gridSlot)
    );
  }
}
