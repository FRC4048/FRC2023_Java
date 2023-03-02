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
    private Action action;

    
    enum Action {
        DoNothing, Balance, PickUpTwo, PickUpOneAndBalance, CrossLine, PickUpOne;
    }

    public AutonomousChooser(Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper) {
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.extender = extender;
        this.gripper = gripper;
        actionChooser = new SendableChooser<Action>();
    }

    public void addOptions() {
        actionChooser.setDefaultOption(Action.DoNothing.name(), Action.DoNothing);
        actionChooser.addOption(Action.Balance.name(), Action.Balance);
        actionChooser.addOption(Action.PickUpTwo.name(), Action.PickUpTwo);
        actionChooser.addOption(Action.PickUpOne.name(), Action.PickUpOne);
        actionChooser.addOption(Action.CrossLine.name(), Action.CrossLine);
        actionChooser.addOption(Action.PickUpOneAndBalance.name(), Action.PickUpOneAndBalance);
        actionChooser.addOption(Action.DoNothing.name(), Action.DoNothing);
    }
    
    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("AutonomousAction", actionChooser);
    }

    public Action getAction() {
        if(actionChooser.getSelected() != null) {
            return actionChooser.getSelected();
        } 
        else {
            return Action.DoNothing;
        }
        
    }


    public Command getAutonomousCommand() {
        action = actionChooser.getSelected();
        if (action == Action.DoNothing) {
            //return new PrintCommand("Do absouletly nothing");
            return new DoNothing(arm, extender);
        }
        else if (action == Action.CrossLine) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.CrossLine) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.CrossLine) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.Balance) {
            //return new PrintCommand("Balance");
            return new EmptyCommand();

        }
        else if (action == Action.Balance){
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();

        }
        else if (action == Action.PickUpTwo) {
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();

        }
        else if (action == Action.PickUpTwo) {
            //return new PrintCommand("Pick up Two");
            return new EmptyCommand();

        }
        else if (action == Action.PickUpTwo) {
            //return new PrintCommand("Pick up Two");
            return new EmptyCommand();

        }
        else if (action == Action.PickUpOne) {
            return new OneGamepiece(drivetrain, arm, extender, gripper);
        }
        else if (action == Action.PickUpOne) {
            return new OneGamepiece(drivetrain, arm, extender, gripper);
        }
        else if (action == Action.PickUpOne) {
            return new OneGamepiece(drivetrain, arm, extender, gripper);

        }
        else if (action == Action.PickUpOneAndBalance) {
            //return new PrintCommand("Cross the line");
            return new EmptyCommand();

        }
        else if (action == Action.PickUpOneAndBalance) {
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();
        }
        else if (action == Action.PickUpOneAndBalance) {
            //return new PrintCommand("Cross the line"); //Invalid
            return new EmptyCommand();
        }
        else {
            //return new PrintCommand("Cross the line");
            return new DoNothing(arm, extender);
        }
    }
}