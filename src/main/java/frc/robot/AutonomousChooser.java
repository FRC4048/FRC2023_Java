package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Autonomous.DoNothing;
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
    private Action action;

    
    enum Action {
        DoNothing, Balence, PickUpTwo, PickUpOneAndBalence, CrossLine, PickUpOne;
    }

    enum Location {
        Right, Middle, Left;
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
        actionChooser.setDefaultOption(Action.DoNothing.name(), Action.DoNothing);
        actionChooser.addOption(Action.Balence.name(), Action.Balence);
        actionChooser.addOption(Action.PickUpTwo.name(), Action.PickUpTwo);
        actionChooser.addOption(Action.PickUpOne.name(), Action.PickUpOne);
        actionChooser.addOption(Action.CrossLine.name(), Action.CrossLine);
        actionChooser.addOption(Action.PickUpOneAndBalence.name(), Action.PickUpOneAndBalence);
        actionChooser.addOption(Action.DoNothing.name(), Action.DoNothing);
        
        locationChooser.setDefaultOption(Location.Middle.name(), Location.Middle);
        locationChooser.addOption(Location.Middle.name(), Location.Middle);
        locationChooser.addOption(Location.Left.name(), Location.Left);
        locationChooser.addOption(Location.Right.name(), Location.Right);
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


    public Command getAutonomousCommand() {
        action = actionChooser.getSelected();
        location = locationChooser.getSelected();
        if (action == Action.DoNothing) {
            //return new PrintCommand("Do absouletly nothing");
            return new DoNothing(arm, extender);
        }
        else if (action == Action.CrossLine && location == Location.Left) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.CrossLine && location == Location.Right) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.CrossLine && location == Location.Middle) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.Balence && location == Location.Middle) {
            //return new PrintCommand("Balence");
            return new EmptyCommand();

        }
        else if (action == Action.Balence && (location == Location.Left || location == Location.Right)){
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();

        }
        else if (action == Action.PickUpTwo && location == Location.Middle) {
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();

        }
        else if (action == Action.PickUpTwo && location == Location.Left) {
            //return new PrintCommand("Pick up Two");
            return new EmptyCommand();

        }
        else if (action == Action.PickUpTwo && location == Location.Right) {
            //return new PrintCommand("Pick up Two");
            return new EmptyCommand();

        }
        else if (action == Action.PickUpOne && location == Location.Left) {
            return new OneGamepiece(drivetrain, arm, extender, gripper);
        }
        else if (action == Action.PickUpOne && location == Location.Right) {
            return new OneGamepiece(drivetrain, arm, extender, gripper);
        }
        else if (action == Action.PickUpOne && location == Location.Middle) {
            return new OneGamepiece(drivetrain, arm, extender, gripper);

        }
        else if (action == Action.PickUpOneAndBalence && location == Location.Middle) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.PickUpOneAndBalence && location == Location.Left) {
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();
        }
        else if (action == Action.PickUpOneAndBalence && location == Location.Right) {
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();
        }
        else {
            //return new PrintCommand("Cross the line");
            return new DoNothing(arm, extender);
        }
    }
}