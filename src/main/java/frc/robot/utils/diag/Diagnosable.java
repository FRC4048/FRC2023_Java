package frc.robot.utils.diag;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface Diagnosable {

    /**
     * Set the tab for the diagnosable (this is done by the diagnostics infrastructure)
     * @param shuffleBoardTab the tab to set
     */
    void setShuffleBoardTab(ShuffleboardTab shuffleBoardTab, int width, int height);

    /**
     * A method called periodically that will test the diagnosable value and update the display
     */
    public void refresh();

    /**
     * A method to reset the "tested" state of the diagnosable component to its initial state
     */
    public void reset();
}
