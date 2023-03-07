package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedPanel;

public class CycleLED {
     
     private final int[] ledSequence;
     int currentImageIndex = 0;
     //includes system time
     double timeSceneImgChange;
     private final LedPanel panel;
     private final double delayInSeconds;

     public CycleLED(LedPanel panel, double delayInSeconds, int... imageIds) {
          this.ledSequence = imageIds;
          this.panel = panel;
          this.delayInSeconds = delayInSeconds;
     }
     
     public void initialize() {
          this.timeSceneImgChange = Timer.getFPGATimestamp();
     }

     public void refresh() {
          if ((Timer.getFPGATimestamp() - timeSceneImgChange) >= delayInSeconds) {
               panel.setID(ledSequence[currentImageIndex]);
               currentImageIndex = (currentImageIndex + 1) % ledSequence.length;
               timeSceneImgChange = Timer.getFPGATimestamp();
          }
     }
     
     
}
