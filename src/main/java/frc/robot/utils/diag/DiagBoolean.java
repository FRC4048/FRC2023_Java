package frc.robot.utils.diag;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Abstract diagnostics class for boolean sensors using DigitalInput
 * The diagnostics will turn green once the switch has changes to reflect both positions: ON and OFF
 */
public abstract class DiagBoolean implements Diagnosable {

    private String title;
    private String name;
    private GenericEntry networkTableEntry;

    private boolean seenFalse;
    private boolean seenTrue;

    /**
     * Constructor
     *
     * @param name            the name of the unit. Will be used on the Shuffleboard
     */
    public DiagBoolean(String title, String name) {
        this.title = title;
        this.name = name;

        reset();
    }

    @Override
    public void setShuffleBoardTab(ShuffleboardTab shuffleBoardTab, int width, int height) {
        networkTableEntry = shuffleBoardTab.getLayout(title, BuiltInLayouts.kList).withSize(width, height).add(name, false).getEntry();
    }

    @Override
    public void refresh() {
        if (networkTableEntry != null) {
            networkTableEntry.setBoolean(getDiagResult());
        }
    }

    @Override
    public void reset() {
        seenFalse = seenTrue = false;
    }

    protected abstract boolean getValue();

    // Package protected
    boolean getDiagResult() {
        boolean currentValue = getValue();
        // Set the value for the state - whether the switch is pressed or not
        if (currentValue) {
            seenTrue = true;
        } else {
            seenFalse = true;
        }

        return seenTrue && seenFalse;
    }

}
