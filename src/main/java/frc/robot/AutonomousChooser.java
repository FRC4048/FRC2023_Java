package frc.robot;

import javax.swing.Action;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
        actionChooser.setDefaultOption(Action.DO_NOTHING.name(), Action.DO_NOTHING);
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

    public Command getAutonomousCommand(Action action) {
        return null;
    }
    


    


    
}
