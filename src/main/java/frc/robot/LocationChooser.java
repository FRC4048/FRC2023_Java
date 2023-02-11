package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.ExampleSubsystem;

public class LocationChooser {
    private SendableChooser<Action> actionChooser;
    private ExampleSubsystem exampleSubsystem;
    private int location;

    enum Action {
        LEFT, MIDDLE, RIGHT;
    }

    public LocationChooser(ExampleSubsystem exampleSubsystem) {
        actionChooser = new SendableChooser<Action>();
        this.exampleSubsystem = exampleSubsystem;
    }

    public void addOptions() {
        actionChooser.setDefaultOption(Action.MIDDLE.name(), Action.MIDDLE);
        actionChooser.addOption(Action.MIDDLE.name(), Action.MIDDLE);
        actionChooser.addOption(Action.LEFT.name(), Action.LEFT);
        actionChooser.addOption(Action.RIGHT.name(), Action.RIGHT);
    }
    
    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("LocationChooser", actionChooser);
    }

    public Action getAction() {
        if(actionChooser.getSelected() != null) {
            return actionChooser.getSelected();
        } 
        else {
            return Action.MIDDLE;
        }
    }

    public int getLocation(Action a) {
        if (a == Action.MIDDLE) {
            location = 0;
            return location;
        }
        else if (a == Action.LEFT) {
            location = -1;
            return location;
        }
        location = 1;
        return location;
    }
}