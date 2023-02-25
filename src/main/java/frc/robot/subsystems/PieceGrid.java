package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Grid;
import frc.robot.utils.SmartShuffleboard;

import java.util.Arrays;

public class PieceGrid extends SubsystemBase {

     private Grid selectedGridSlot = Grid.MIDDLE_MIDDLE;

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
          Arrays.stream(Grid.values()).forEach(grid -> SmartShuffleboard.put("Driver",grid.name(),isSlotSelected(grid)));
     }


     public boolean isSlotSelected(Grid slot) {
          return slot.equals(selectedGridSlot);
     }

     public Grid getSelectedGridSlot() {
          return selectedGridSlot;
     }

     /**
      * sets the
      * @param slot the target slot
      */
     public void setSelectedGridSlot(Grid slot) {
          SmartShuffleboard.put("Driver",selectedGridSlot.name(),false);
          selectedGridSlot = slot;
          SmartShuffleboard.put("Driver",selectedGridSlot.name(),true);
     }
}
