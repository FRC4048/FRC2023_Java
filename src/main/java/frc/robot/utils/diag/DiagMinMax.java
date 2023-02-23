package frc.robot.utils.diag;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Abstract diagnostics class for devices that need a minimum and maximum reading to be tested. 
 * The diagnostics will turn green once the device has reached both points.
 */
public abstract class DiagMinMax implements Diagnosable {

    private String name;
    private String title;
    private double minValue;
    private double maxValue;
    private GenericEntry networkTableEntry;

    private boolean seenMinValue;
    private boolean seenMaxValue;

    /**
     * Constructor
     *
     * @param name            - the name of the unit. Will be used on the Shuffleboard
     * @param minValue        - the minimum value the object needs to hit to qualify for success
     * @param maxValue        - the maximum value the object needs to hit to qualify for success
     */
    public DiagMinMax(String title, String name, double minValue, double maxValue) {
        this.name = name;
        this.title = title;
        this.minValue = minValue;
        this.maxValue = maxValue;

        reset();
    }

    @Override
    public void setShuffleBoardTab(ShuffleboardTab shuffleBoardTab, int width, int height) {
        networkTableEntry = shuffleBoardTab.getLayout(title, BuiltInLayouts.kList).withSize(width, height).add(name, false).getEntry();
    }

    @Override
    public void refresh() {
        if (networkTableEntry != null) {
            networkTableEntry.setBoolean(getDiagResult(getSensorReading()));
        }
    }

    @Override
    public void reset() {
        seenMinValue = seenMaxValue = false;
    }

    abstract double getSensorReading();

    boolean getDiagResult(double value) {
        if (value >= maxValue) {
            seenMaxValue = true;
        } else if (value <= minValue) {
            seenMinValue = true;
        }

        return seenMaxValue && seenMinValue;
    }
}
