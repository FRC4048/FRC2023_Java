package frc.robot.utils.diag;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Base class for Diagnosable that have an initial value (their "current" value) and that are required to change their
 * value by a certain amount from that
 */
public abstract class DiagDistanceTraveled implements Diagnosable {

    protected String name;
    protected String title;
    protected double requiredTravel;
    protected GenericEntry networkTableEntry;
    private double initialValue;
    private boolean traveledDistance;


    public DiagDistanceTraveled(String title, String name, double requiredTravel) {
        this.title = title;
        this.name = name;
        this.requiredTravel = requiredTravel;
    }

    @Override
    public void setShuffleBoardTab(ShuffleboardTab shuffleBoardTab, int width, int height) {
        networkTableEntry = shuffleBoardTab.getLayout(title, BuiltInLayouts.kList).withSize(width, height).add(name, false).getEntry(); //getLayout(title, BuiltInLayouts.kList)
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
        double currentValue = getCurrentValue();

        if (Math.abs(currentValue - initialValue) >= requiredTravel) {
            traveledDistance = true;
        }

        return this.traveledDistance;
    }

    protected abstract double getCurrentValue();
}
