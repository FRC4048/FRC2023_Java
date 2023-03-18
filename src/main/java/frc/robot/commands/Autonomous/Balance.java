package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonomousChooser;
import frc.robot.commands.sequences.AutoBalanceSequence;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;

public class Balance extends SequentialCommandGroup {

    public Balance(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper, AutonomousChooser.Location location) {
        setName("BalanceSequence");

        addCommands(
            new SequentialCommandGroupWrapper(new ResetEncoders(arm, extender)),
                new SequentialCommandGroupWrapper(new AutoBalanceSequence(drivetrain, arm, extender))
        );
    }
}
