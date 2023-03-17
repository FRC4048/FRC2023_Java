package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonomousChooser;
import frc.robot.commands.BalancePID;
import frc.robot.commands.sequences.AutoBalanceSequence;
import frc.robot.commands.sequences.PIDBalanceSequence;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class Balance extends SequentialCommandGroup {

    public Balance(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper, AutonomousChooser.Location location) {

        addCommands(
                new ResetEncoders(arm, extender),
                new PIDBalanceSequence(drivetrain, true)
        );
    }
}
