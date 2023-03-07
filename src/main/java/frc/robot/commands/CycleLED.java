package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedPanel;

public class CycleLED extends CommandBase {
     
     private final int[] ledSequence;
     int currentImageIndex = 0;
     //includes system time
     long timeSceneImgChange;
     private final LedPanel panel;
     private final int delayInMilliSeconds;

     public CycleLED(LedPanel panel, int delayInMilliSeconds, int... imageIds) {
          this.ledSequence = imageIds;
          this.panel = panel;
          this.delayInMilliSeconds = delayInMilliSeconds;
          addRequirements(panel);
     }

     @Override
     public void initialize() {
          this.timeSceneImgChange = System.currentTimeMillis();
     }

     @Override
     public void execute() {
          if (System.currentTimeMillis() - timeSceneImgChange >= delayInMilliSeconds) {
               panel.setID(ledSequence[currentImageIndex]);
               if (currentImageIndex== ledSequence.length-1) currentImageIndex= 0;
               else currentImageIndex++;
               timeSceneImgChange = System.currentTimeMillis();
          }
     }
     

     @Override
     public boolean isFinished() {
          return false;
     }
}
