package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AprilTagPosition;
import frc.robot.subsystems.Drivetrain;

public class SetAprilTagRelPosition extends SequentialCommandGroup{
    public SetAprilTagRelPosition (Drivetrain drivetrain, AprilTagPosition apriltag, double desiredVertical, double desiredHorizontal, double desiredRotation) {
        addCommands(
            
        new ParallelCommandGroup(
        
            new AprilTagSetHorizontal(drivetrain, desiredHorizontal, apriltag),
            new AprilTagSetVertical(drivetrain, desiredVertical, apriltag),
            new AprilTagSetRotation(drivetrain, desiredRotation, apriltag)
            )
        );
    }
    
    
    
}
