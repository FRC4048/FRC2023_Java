/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

/**
 * Add your docs here.
 */
public class AutoChooser {
    private SendableChooser<Action> actionChooser;
   
    // all actions driver chooses at beginning of match
    enum Action {
        
    }

    public AutoChooser() {
        actionChooser = new SendableChooser<Action>();
        
    }

    public void addOptions() {
        
        // Last years auto options, new commands must be imported once made
         /* 
         actionChooser.setDefaultOption(Action.TWO_SHOT_RIGHT.name(), Action.TWO_SHOT_RIGHT);
         actionChooser.addOption(Action.THREE_SHOT_RIGHT.name(), Action.THREE_SHOT_RIGHT); 
         actionChooser.addOption(Action.TWO_SHOT_LEFT.name(), Action.TWO_SHOT_LEFT);
         actionChooser.addOption(Action.ONE_SHOT.name(), Action.ONE_SHOT);
         actionChooser.addOption(Action.CROSS_LINE.name(), Action.CROSS_LINE);
         actionChooser.addOption(Action.DO_NOTHING.name(), Action.DO_NOTHING);
         */

    }

    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Action", actionChooser);
    }

    /* 
     public Action getAction(){
        if(actionChooser.getSelected() != null){
            return actionChooser.getSelected();
        } else{
            //return default action
        }
     } */
/* 
    public Command getAutonomousCommand(Action a) {
        
        
    }*/
}
