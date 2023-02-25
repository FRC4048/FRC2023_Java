package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmPositionGrid;
import frc.robot.utils.SmartShuffleboard;

import java.util.Arrays;

public class PieceGrid extends SubsystemBase {

     private ArmPositionGrid selectedGridSlot = ArmPositionGrid.MIDDLE_MIDDLE;
     private boolean gridCreated = false;

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
          Arrays.stream(ArmPositionGrid.values()).forEach(grid -> SmartShuffleboard.put("Driver",grid.name(),isSlotSelected(grid)));
          gridCreated = false;
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
          if (!gridCreated) setupGrid();
          SmartShuffleboard.put("Driver",selectedGridSlot.name(),false);
          selectedGridSlot = slot;
          SmartShuffleboard.put("Driver",selectedGridSlot.name(),true);
     }
}
