package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class MoveDistanceOffsetTest extends SequentialCommandGroup{
    public MoveDistanceOffsetTest (Drivetrain drivetrain) {

        addCommands(
            new MoveDistanceOffset(drivetrain, 4, 0, Math.toRadians(180))
        );

    }
}
