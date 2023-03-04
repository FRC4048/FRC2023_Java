package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedPanel;

public class ChangeLedID extends CommandBase{

    public ChangeLedID(LedPanel ledPanel, int change) {
        LedPanel.setID(LedPanel.getID() + change);
    }
}
