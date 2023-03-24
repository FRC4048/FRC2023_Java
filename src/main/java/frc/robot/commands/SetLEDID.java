package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedPanel;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class SetLEDID extends LoggedCommand {
    LedPanel ledpanel;
    int setID;
    public SetLEDID(LedPanel ledpanel, int setID) {
        this.ledpanel = ledpanel;
        this.setID = setID;
        addRequirements(ledpanel);
    }

    @Override
    public void initialize() {
        super.initialize();
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
