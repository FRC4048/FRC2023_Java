package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonomousChooser;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;

public class CrossTheLine extends SequentialCommandGroup {
    
    public CrossTheLine (Drivetrain drivetrain, Arm arm, Extender extender, AutonomousChooser.Location location) {
        setName("-Auto-CTL");
        addCommands(
            new SequentialCommandGroupWrapper(new ResetEncoders(arm, extender),"-Auto-Reset-Encoders"),
            new MoveDistanceSpinTraj(drivetrain, 4, 0, Math.toRadians(180))
        );

    }
}
