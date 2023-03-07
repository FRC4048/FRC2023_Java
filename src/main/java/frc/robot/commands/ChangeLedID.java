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
        ledPanel.setID(ledPanel.getID() + change);
        if (ledPanel.getID() > 7) {
            ledPanel.setID(0);
        }
    }

    public boolean isFinished() {
        return true;
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
        } 

    
}
