package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Autonomous.EmptyCommand;
import frc.robot.commands.Autonomous.OneGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class AutonomousChooser {

    private Drivetrain drivetrain;
    private Arm arm;
    private Extender extender;
    private GripperSubsystem gripper;
    private SendableChooser<Action> actionChooser;
    private SendableChooser<Location> locationChooser;
    private Location location;

    enum Action {
        DoNothing, Balence, PickUpTwo, PickUpOneAndBalence, CrossLine, PickUpOne;
    }

    enum Location {
        NextToWall, Middle, NextToSubtation;
    }

    public AutonomousChooser(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper) {
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.extender = extender;
        this.gripper = gripper;
        actionChooser = new SendableChooser<Action>();
        locationChooser = new SendableChooser<Location>();
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
        tab.add("AutonomousAction", actionChooser);

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

    public Location getLocation() {
        if (locationChooser.getSelected() != null) {
            return locationChooser.getSelected();
        }
        else {
            return Location.Middle;
        }
    }
    public Command getAutonomousCommand(Action action) {
        // if (action == Action.DoNothing) {
        //     //return new PrintCommand("Do absouletly nothing");
        // }
        // else if (action == Action.CrossLine && location == Location.NextToSubtation) {
        //     //return new PrintCommand("Cross the line");
        // }
        // else if (action == Action.CrossLine && location == Location.NextToWall) {
        //     //return new PrintCommand("Cross the line");
        // }
        // else if (action == Action.CrossLine && location == Location.Middle) {
        //     //return new PrintCommand("Cross the line");
        // }
        // else if (action == Action.Balence && location == Location.Middle) {
        //     //return new PrintCommand("Balence");
        // }
        // else if (action == Action.Balence && (location == Location.NextToSubtation || location == Location.NextToWall)){
        //     //return new PrintCommand("Cross the line"); //Invalid
        // }
        // else if (action == Action.PickUpTwo && location == Location.Middle) {
        //     //return new PrintCommand("Cross the line"); //Invalid
        // }
        // else if (action == Action.PickUpTwo && location == Location.NextToSubtation) {
        //     //return new PrintCommand("Pick up Two");
        // }
        // else if (action == Action.PickUpTwo && location == Location.NextToWall) {
        //     //return new PrintCommand("Pick up Two");
        // }
        // else if (action == Action.PickUpOne && location == Location.NextToSubtation) {
        //     new OneGamepiece(drivetrain, arm, extender, gripper);
        // }
        // else if (action == Action.PickUpOne && location == Location.NextToWall) {
        //     //return new PrintCommand("Pick Up One");
        // }
        // else if (action == Action.PickUpOne && location == Location.Middle) {
        //     //return new PrintCommand("Pick Up One");
        // }
        // else if (action == Action.PickUpOneAndBalence && location == Location.Middle) {
        //     //return new PrintCommand("Cross the line");
        // }
        // else if (action == Action.PickUpOneAndBalence && location == Location.NextToSubtation) {
        //     //return new PrintCommand("Cross the line"); //Invalid
        //     return new EmptyCommand();
        // }
        // else if (action == Action.PickUpOneAndBalence && location == Location.NextToWall) {
        //     //return new PrintCommand("Cross the line"); //Invalid
        //     return new EmptyCommand();
        // }
        // else {
        //     //return new PrintCommand("Cross the line");
        //     return new EmptyCommand();
        // }
            return new OneGamepiece(drivetrain, arm, extender, gripper);
    }
}