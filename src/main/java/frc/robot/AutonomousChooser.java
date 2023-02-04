package frc.robot;

import javax.swing.Action;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Cube;
import frc.robot.commands.Autonomous.Cone;
import frc.robot.commands.Autonomous.DoNothing;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class AutonomousChooser {
    private SendableChooser<Action> actionChooser;
    private ExampleSubsystem exampleSubsystem;

    enum Action {
        CUBE, CONE, DONOTHING;
    }

    public AutonomousChooser(ExampleSubsystem exampleSubsystem) {
        actionChooser = new SendableChooser<Action>();
        this.exampleSubsystem = exampleSubsystem;
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
            return new Cube(exampleSubsystem);
        }
        else if (a == Action.CONE) {
            return new Cone(exampleSubsystem);
        }
        return new DoNothing(exampleSubsystem);
    }
}