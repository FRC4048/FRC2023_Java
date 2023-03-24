package frc.robot.commands;

import frc.robot.subsystems.LedPanel;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ChangeLedID extends LoggedCommand{
    LedPanel ledPanel; 
    int change;
    public ChangeLedID(LedPanel ledPanel, int change) {
        this.ledPanel = ledPanel;
        this.change = change;
        addRequirements(ledPanel);
    }


    @Override
    public void initialize() {
        super.initialize();
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
