package frc.robot.commands.sequences;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class TestTraj extends SequentialCommandGroup{

    public TestTraj(Drivetrain drivetrain) {
        PathPlannerTrajectory newPath = PathPlanner.loadPath("Test3", new PathConstraints(2.5, 5));
        addCommands(
            drivetrain.followTrajectoryCommand(newPath, true)
        );

    }
       
        
    }
