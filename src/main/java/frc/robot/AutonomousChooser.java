package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Autonomous.CrossTheLine;
import frc.robot.commands.Autonomous.DoNothing;
import frc.robot.commands.Autonomous.OneGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;

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
        //Balance, 
        //DepositTwo, 
        //DepositOneAndBalance
        DoNothing, 
        CrossLine,  
        OnePieceMoveLeft, 
        OnePieceMoveRight;
    }

    public enum Location {	
        Right, 
        Middle, 
        Left;	
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
        actionChooser.setDefaultOption("Do Nothing", Action.DoNothing);
        //actionChooser.addOption("Balance", Action.Balance);
        //actionChooser.addOption("Drop Two Pieces", Action.DepositTwo);
        actionChooser.addOption("One Piece Move Left", Action.OnePieceMoveLeft);
        actionChooser.addOption("One Piece Move Right", Action.OnePieceMoveRight);
        actionChooser.addOption("Cross the Line", Action.CrossLine);
        //actionChooser.addOption("Pick Up One and Balance", Action.DepositOneAndBalance);

        locationChooser.setDefaultOption(Location.Middle.name(), Location.Middle);	
        locationChooser.addOption(Location.Left.name(), Location.Left);	
        locationChooser.addOption(Location.Right.name(), Location.Right);
    }
    
    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Action", actionChooser);
        tab.add("Location Chooser", locationChooser);
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
        Alliance allianceColor = DriverStation.getAlliance();

        if (action == Action.DoNothing) {
            return new SequentialCommandGroupWrapper(new DoNothing(arm, extender));
        }
        else if (action == Action.CrossLine && location != Location.Middle) {
            return new SequentialCommandGroupWrapper(new CrossTheLine(drivetrain, arm, extender, location, allianceColor));
        }
        else if (action == Action.OnePieceMoveLeft) {
            return new SequentialCommandGroupWrapper(new OneGamepiece(drivetrain, arm, extender, gripper, 1, location, allianceColor));
        }
        else if (action == Action.OnePieceMoveRight) {
            return new SequentialCommandGroupWrapper(new OneGamepiece(drivetrain, arm, extender, gripper, -1, location, allianceColor));
        }
        else {
            return new SequentialCommandGroupWrapper(new DoNothing(arm, extender));
        }
    }
}