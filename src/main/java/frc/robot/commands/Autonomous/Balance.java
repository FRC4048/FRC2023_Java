package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonomousChooser;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class Balance extends SequentialCommandGroup {
    
    public Balance (Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper, AutonomousChooser.Location location, Alliance allianceColor) {

        addCommands(
            new ResetEncoders(arm, extender),
            new MoveDistanceSpinTraj(drivetrain, 0, 3, Math.toRadians(180)),
            new AutoBalance(drivetrain)
        );

    }
}
