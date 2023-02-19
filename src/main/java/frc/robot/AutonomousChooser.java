package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Cube;
import frc.robot.commands.Autonomous.Cone;
import frc.robot.commands.Autonomous.DoNothing;
import frc.robot.subsystems.Drivetrain;

public class AutonomousChooser {
    private SendableChooser<Action> actionChooser;
    private Drivetrain drivetrain;

    enum Action {
        CUBE, CONE, DONOTHING;
    }

    public AutonomousChooser(Drivetrain drivetrain) {
        actionChooser = new SendableChooser<Action>();
        this.drivetrain = drivetrain;
    }

    public void addOptions() {
        actionChooser.setDefaultOption(Action.CUBE.name(), Action.CUBE);
        actionChooser.addOption(Action.CUBE.name(), Action.CUBE);
        actionChooser.addOption(Action.CONE.name(), Action.CONE);
        actionChooser.addOption(Action.DONOTHING.name(), Action.DONOTHING);
    }
    
    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Action", actionChooser);
    }

    public Action getAction() {
        if(actionChooser.getSelected() != null) {
            return actionChooser.getSelected();
        } 
        else {
            return Action.DONOTHING;
        }
    }

    public SequentialCommandGroup getAutonomousCommand(Action a) {
        if (a == Action.CUBE) {
            return new Cube(drivetrain);
        }
        else if (a == Action.CONE) {
            return new Cone(drivetrain);
        }
        return new DoNothing(drivetrain);
    }
}