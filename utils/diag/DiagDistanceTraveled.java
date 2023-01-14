package frc.robot.utils.diag;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Base class for Diagnosable that have an initial value (their "current" value) and that are required to change their
 * value by a certain amount from that
 */
public abstract class DiagDistanceTraveled implements Diagnosable {

    protected String name;
    protected int requiredTravel;
    protected NetworkTableEntry networkTableEntry;
    private int initialValue;
    private boolean traveledDistance;


    public DiagDistanceTraveled(String name, int requiredTravel) {
        this.name = name;
        this.requiredTravel = requiredTravel;
    }

    @Override
    public void setShuffleBoardTab(ShuffleboardTab shuffleBoardTab) {
        networkTableEntry = shuffleBoardTab.add(name, false).getEntry();
    }

    @Override
    public void refresh() {
        if (networkTableEntry != null) {
            networkTableEntry.setBoolean(getDiagResult());
        }
    }

    @Override
    public void reset() {
        traveledDistance = false;
        initialValue = getCurrentValue();
    }

    boolean getDiagResult() {
        int currentValue = getCurrentValue();

        if (Math.abs(currentValue - initialValue) >= requiredTravel) {
            traveledDistance = true;
        }

        return this.traveledDistance;
    }

    protected abstract int getCurrentValue();
}
