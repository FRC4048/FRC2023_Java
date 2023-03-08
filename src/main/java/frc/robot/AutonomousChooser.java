package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autonomous.Balance;
import frc.robot.commands.Autonomous.CrossTheLine;
import frc.robot.commands.Autonomous.DepositOneAndBalance;
import frc.robot.commands.Autonomous.DoNothing;
import frc.robot.commands.Autonomous.OneGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

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
        Balance, 
        //DepositTwo, 
        DepositOneAndBalanceRight,
        DepositOneAndBalanceLeft,
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
        actionChooser.addOption("One Piece Move Right Balance", Action.DepositOneAndBalanceRight);
        actionChooser.addOption("One Piece Move Left Balance", Action.DepositOneAndBalanceLeft);
        actionChooser.addOption("Balance", Action.Balance);

        locationChooser.setDefaultOption(Location.Middle.name(), Location.Middle);	
        locationChooser.addOption(Location.Left.name(), Location.Left);	
        locationChooser.addOption(Location.Right.name(), Location.Right);
    }

    public void setOdometry(Drivetrain drivetrain, Location location, Action action, Alliance alliance) {
        double x = 0;
        double y = 0;
        if (alliance == Alliance.Blue) {
            
            if(action == Action.CrossLine && location == Location.Left) {
                x = 72;
                y = 186;
            }
            else if(action == Action.CrossLine && location == Location.Middle) {
                x = 72;
                y = 108;
            }
            else if(action == Action.CrossLine && location == Location.Right) {
                x = 72;
                y = 30;
            }
            

            else if(action == Action.Balance) {
                x = 72;
                y = 108;
            }
            
            
            else if(action == Action.DepositOneAndBalanceRight) {
                x = 72;
                y = 130;
            }


            else if(action == Action.DepositOneAndBalanceLeft) {
                x = 72;
                y = 86;
            }

            
            else if(action == Action.OnePieceMoveLeft && location == Location.Right) {
                x = 72;
                y = 20;
            }
            else if(action == Action.OnePieceMoveLeft && location == Location.Left) {
                x = 72;
                y = 150;
            }
            
            
            else if(action == Action.OnePieceMoveRight && location == Location.Right) {
                x = 72;
                y = 64;
            }
            else if(action == Action.OnePieceMoveRight && location == Location.Left) {
                x = 72;
                y = 194;
            }

            else {
                x = 72;
                y = 108;
            }
        }
        
        
        
        else if (alliance == Alliance.Red) {
            
            if(action == Action.CrossLine && location == Location.Left) {
                x = 72;
                y = 285;
            }
            else if(action == Action.CrossLine && location == Location.Middle) {
                x = 72;
                y = 207;
            }
            else if(action == Action.CrossLine && location == Location.Right) {
                x = 72;
                y = 129;
            }
            

            else if(action == Action.Balance) {
                x = 72;
                y = 207;
            }
            
            
            else if(action == Action.DepositOneAndBalanceRight) {
                x = 72;
                y = 229;
            }


            else if(action == Action.DepositOneAndBalanceLeft) {
                x = 72;
                y = 185;
            }

            
            else if(action == Action.OnePieceMoveLeft && location == Location.Right) {
                x = 72;
                y = 119;
            }
            else if(action == Action.OnePieceMoveLeft && location == Location.Left) {
                x = 72;
                y = 249;
            }
            
            
            else if(action == Action.OnePieceMoveRight && location == Location.Right) {
                x = 72;
                y = 163;
            }
            else if(action == Action.OnePieceMoveRight && location == Location.Left) {
                x = 72;
                y = 293;
            }

            else {
                x = 72;
                y = 207;
            }
            
        }
        
        drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(x), Units.inchesToMeters(y), new Rotation2d(Math.toRadians(180))));
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

        setOdometry(drivetrain, location, action, allianceColor);

        if (action == Action.DoNothing) {
            return new DoNothing(arm, extender);
        }
        else if (action == Action.CrossLine && location != Location.Middle) {
            return new CrossTheLine(drivetrain, arm, extender, location, allianceColor);
        }
        else if (action == Action.OnePieceMoveLeft) {
            return new OneGamepiece(drivetrain, arm, extender, gripper, 1, location, allianceColor);
        }
        else if (action == Action.OnePieceMoveRight) {
            return new OneGamepiece(drivetrain, arm, extender, gripper, -1, location, allianceColor);
        }
        else if (action == Action.DepositOneAndBalanceRight) {
            return new DepositOneAndBalance(drivetrain, arm, extender, gripper, -1, location, allianceColor);
        }
        else if (action == Action.DepositOneAndBalanceLeft) {
            return new DepositOneAndBalance(drivetrain, arm, extender, gripper, 1, location, allianceColor);
        }
        else if (action == Action.Balance) {
            return new Balance(drivetrain, arm, extender, gripper, location, allianceColor);
        }
        else {
            return new DoNothing(arm, extender);
        }
    }
}