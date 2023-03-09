package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmPositionGrid;
import frc.robot.utils.SmartShuffleboard;

public class PieceGrid extends SubsystemBase {

     private ArmPositionGrid selectedGridSlot = ArmPositionGrid.MIDDLE_MIDDLE;
     ArmPositionGrid grid;

     public PieceGrid() {
          setupGrid();
     }

     @Override
     public void periodic() {

     }

     /**
      * creates a selection grid on shuffleboard which shows the selected game piece target position
      */
     private void setupGrid() {
          int j = 0;
          for (int i = 0; i < ArmPositionGrid.values().length; i++) {
               if (i%3 == 0) {
                    j++;
               }
               grid = ArmPositionGrid.values()[i];
               SmartShuffleboard.put("Driver", grid.name(), isSlotSelected(grid)).withPosition(7+i%3, j-1);
          }
     }


     public boolean isSlotSelected(ArmPositionGrid slot) {
          return slot.equals(selectedGridSlot);
     }

     public ArmPositionGrid getSelectedGridSlot() {
          return selectedGridSlot;
     }

     /**
      * sets the
      * @param slot the target slot
      */
     public void setSelectedGridSlot(ArmPositionGrid slot) {
          selectedGridSlot = slot;
          setupGrid();
     }
}
