package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedPanel;

public class SetLEDID extends CommandBase {
    LedPanel ledpanel;
    int setID;
    public SetLEDID(LedPanel ledpanel, int setID) {
        this.ledpanel = ledpanel;
        this.setID = setID;
    }

    @Override
    public void initialize() {
        ledpanel.setID(setID);
    }

    public boolean isFinished() {
        return true;
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
        } 

    
}
