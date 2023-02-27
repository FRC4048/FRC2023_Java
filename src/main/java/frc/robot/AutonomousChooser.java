package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutonomousChooser {

    private SendableChooser<Action> actionChooser;
    private SendableChooser<Location> locationChooser;
    private Location location;
    
    


    enum Action {
        DoNothing, Balence, PickUpTwo, PickUpOneAndBalence, CrossLine, PickUpOne;
    }

    enum Location {
        NextToWall, Middle, NextToSubtation;
    }

    public AutonomousChooser() {
        actionChooser = new SendableChooser<Action>();
    }

    public void LocationChooser() {
        locationChooser = new SendableChooser<Location>();
    }

    public Location getLocation() {
        if(locationChooser.getSelected() != null) {
            return locationChooser.getSelected();
        } 
        else {
            return Location.Middle;
        }
    }

    public void addOptions() {
        actionChooser.setDefaultOption(Action.CrossLine.name(), Action.CrossLine);
        actionChooser.addOption(Action.Balence.name(), Action.Balence);
        actionChooser.addOption(Action.PickUpTwo.name(), Action.PickUpTwo);
        actionChooser.addOption(Action.PickUpOne.name(), Action.PickUpOne);
        actionChooser.addOption(Action.CrossLine.name(), Action.CrossLine);
        actionChooser.addOption(Action.PickUpOneAndBalence.name(), Action.PickUpOneAndBalence);
        actionChooser.addOption(Action.DoNothing.name(), Action.DoNothing);
        
        locationChooser.setDefaultOption(Location.Middle.name(), Location.Middle);
        locationChooser.addOption(Location.Middle.name(), Location.Middle);
        locationChooser.addOption(Location.NextToSubtation.name(), Location.NextToSubtation);
        locationChooser.addOption(Location.NextToWall.name(), Location.NextToWall);
    }
    
    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Action", actionChooser);
        tab.add("LocationChooser", locationChooser);
    }

    public Action getAction() {
        if(actionChooser.getSelected() != null) {
            return actionChooser.getSelected();
        } 
        else {
            return Action.DoNothing;
        }
        
    }

    public Location getLocation(Location action) {
        if (action == Location.Middle) {
            location = Location.Middle;
            return location;
        }
        else if (action == Location.NextToSubtation) {
            location = Location.NextToSubtation;
            return location;
        }
        else {
            location = Location.NextToWall;
            return location;
        }
    }
    public PrintCommand getAutonomousCommand(Action action) {
        if (action == Action.DoNothing) {
            return new PrintCommand("Do absouletly nothing");
        }
        else if (action == Action.CrossLine && location == Location.NextToSubtation) {
            return new PrintCommand("Cross the line");
        }
        else if (action == Action.Balence && location == Location.Middle) {
            return new PrintCommand("Balence");
        }
        else if (action == Action.Balence && (location == Location.NextToSubtation || location == Location.NextToWall)){
            return new PrintCommand("Cross the line");
        }
        else if (action == Action.PickUpTwo && location == Location.Middle) {
            return new PrintCommand("Cross the line");
        }
        else if (action == Action.PickUpTwo && location == Location.NextToSubtation || location == Location.NextToWall) {
            return new PrintCommand("Pick up Two");
        }
        else if (action == Action.PickUpOne && location == Location.Middle) {
            return new PrintCommand("i");
        }
        else {
            return new PrintCommand("Cross the line");
        }

    }
}