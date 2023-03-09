package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedPanel;

public class ChangeLedID extends CommandBase{
    LedPanel ledPanel; 
    int change;
    public ChangeLedID(LedPanel ledPanel, int change) {
        this.ledPanel = ledPanel;
        this.change = change;
        addRequirements(ledPanel);
    }


    @Override
    public void initialize() {
        int nextID = ledPanel.getID();
        nextID += change;
        if (nextID > 7) {
            nextID = 0;
        }
        ledPanel.setID(nextID);
    }

    public boolean isFinished() {
        return true;
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
        } 

    
}
