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
import frc.robot.commands.Autonomous.TwoGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
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
        Balance, 
        TwoPieceMoveLeft,
        TwoPieceMoveRight, 
        //DepositTwo, 
        DepositOneAndBalance,
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
        //actionChooser.addOption("Drop Two Pieces", Action.DepositTwo);
        actionChooser.addOption("One Piece Move Left", Action.OnePieceMoveLeft);
        actionChooser.addOption("One Piece Move Right", Action.OnePieceMoveRight);
        actionChooser.addOption("Cross the Line", Action.CrossLine);
        actionChooser.addOption("One Piece and Balance", Action.DepositOneAndBalance);
        actionChooser.addOption("Balance", Action.Balance);
        actionChooser.addOption("Two Piece Move Right", Action.TwoPieceMoveRight);
        actionChooser.addOption("Two Piece Move Left", Action.TwoPieceMoveLeft);

        locationChooser.setDefaultOption(Location.Middle.name(), Location.Middle);	
        locationChooser.addOption(Location.Left.name(), Location.Left);	
        locationChooser.addOption(Location.Right.name(), Location.Right);
    }

    public void setOdometry(Drivetrain drivetrain, Location location, Action action, Alliance alliance) {
        double x = 72;
        double y = 0;

        if (action == Action.CrossLine && location == Location.Left) {
            y = 186;
        } else if (action == Action.CrossLine && location == Location.Middle) {
            y = 108;
        } else if (action == Action.CrossLine && location == Location.Right) {
            y = 30;
        } else if (action == Action.Balance) {
            y = 108;
        } else if (action == Action.DepositOneAndBalance) {
            y = 108;
        } else if (action == Action.OnePieceMoveLeft && location == Location.Right) {
            y = 20;
        } else if (action == Action.OnePieceMoveLeft && location == Location.Left) {
            y = 150;
        } else if (action == Action.OnePieceMoveRight && location == Location.Right) {
            y = 64;
        } else if (action == Action.OnePieceMoveRight && location == Location.Left) {
            y = 194;
        } else if (action == Action.TwoPieceMoveLeft && location == Location.Left) {
            y = 150;
        } else if (action == Action.TwoPieceMoveRight && location == Location.Left) {
            y = 194;
        } else if (action == Action.TwoPieceMoveRight && location == Location.Right) {
            y = 64;
        } else if (action == Action.TwoPieceMoveLeft && location == Location.Right) {
            y = 20;
        } else {
            y = 108;
        }

        if (alliance == Alliance.Red) {
            y += 99;
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
            return new SequentialCommandGroupWrapper(new DoNothing(arm, extender, drivetrain));
        }
        else if (action == Action.CrossLine) {
            if ((location == Location.Right) || (location == Location.Left)) {
                return new SequentialCommandGroupWrapper(new CrossTheLine(drivetrain, arm, extender, location));
            }
            else {
                return new SequentialCommandGroupWrapper(new DoNothing(arm, extender, drivetrain));
            }
        }
        else if (action == Action.OnePieceMoveLeft) {
            if (location == Location.Right) {
                return new SequentialCommandGroupWrapper(new OneGamepiece(drivetrain, arm, extender, gripper, 0.2, location));
            }
            else if (location == Location.Left) {
                return new SequentialCommandGroupWrapper(new OneGamepiece(drivetrain, arm, extender, gripper, 0.6, location));
            }
            else {
                return new SequentialCommandGroupWrapper(new DoNothing(arm, extender, drivetrain));
            }
        }
        else if (action == Action.OnePieceMoveRight) {
            if (location == Location.Right) {
                return new SequentialCommandGroupWrapper(new OneGamepiece(drivetrain, arm, extender, gripper, -0.6, location));
            }
            else if (location == Location.Left) {
                return new SequentialCommandGroupWrapper(new OneGamepiece(drivetrain, arm, extender, gripper, -0.2, location));
            }
            else {
                return new SequentialCommandGroupWrapper(new DoNothing(arm, extender, drivetrain));
            }
        }
        else if (action == Action.TwoPieceMoveLeft) {
            if (location == Location.Left && allianceColor == Alliance.Blue) {
                return new TwoGamepiece(drivetrain, arm, extender, gripper, -1);
            }
            else if (location == Location.Right && allianceColor == Alliance.Red) {
                return new TwoGamepiece(drivetrain, arm, extender, gripper, -1);
            }
            else {
                return new DoNothing(arm, extender, drivetrain);
            }
        }
        else if (action == Action.TwoPieceMoveRight) {
            if (location == Location.Left && allianceColor == Alliance.Blue) {
                return new TwoGamepiece(drivetrain, arm, extender, gripper, 1);
            }
            else if (location == Location.Right && allianceColor == Alliance.Red) {
                return new TwoGamepiece(drivetrain, arm, extender, gripper, 1);
            }
            else {
                return new DoNothing(arm, extender, drivetrain);
            }
        }
        else if (action == Action.DepositOneAndBalance && location == Location.Middle) {
            return new SequentialCommandGroupWrapper(new DepositOneAndBalance(drivetrain, arm, extender, gripper, -0.2, location));
        }
        else if (action == Action.Balance && location == Location.Middle) {
            return new SequentialCommandGroupWrapper(new Balance(drivetrain, arm, extender, gripper, location));
        }
        else {
            return new SequentialCommandGroupWrapper(new DoNothing(arm, extender, drivetrain));
        }
    }
}