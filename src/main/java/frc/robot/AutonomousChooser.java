package frc.robot;

import javax.swing.Action;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autonomous.Cube;
import frc.robot.commands.Autonomous.Cone;
import frc.robot.commands.Autonomous.DO_NOTHING;
import frc.robot.subsystems.ExampleSubsystem;

public class AutonomousChooser {
    private SendableChooser<Action> actionChooser;
    private ExampleSubsystem exampleSubsystem;

    enum Action {
        CUBE, CONE, DO_NOTHING;
    }

    public AutonomousChooser(ExampleSubsystem exampleSubsystem) {
        actionChooser = new SendableChooser<Action>();
        this.exampleSubsystem = exampleSubsystem;
    }

    public void addOptions() {
        actionChooser.setDefaultOption(Action.CUBE.name(), Action.CUBE);
        actionChooser.addOption(Action.CUBE.name(), Action.CUBE);
        actionChooser.addOption(Action.CONE.name(), Action.CONE);
        actionChooser.addOption(Action.DO_NOTHING.name(), Action.DO_NOTHING);
    }
    
    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Action", actionChooser);
    }

    public Action getAction() {
        if(actionChooser.getSelected() != null){
            return actionChooser.getSelected();
        } else{
            return Action.DO_NOTHING;
        }
    }

    public Command getAutonomousCommand(Action a) {
        return null;
        /*
        if (a == Action.CUBE) {
            return new Cube(exampleSubsystem);
        }
        if (a == Action.CONE) {
            return new Cone(exampleSubsystem);
        }
        
        if (a == Action.DO_NOTHING) {
            return new DO_NOTHING(exampleSubsystem);
        }
        */
    }
}
    
    



    


    

