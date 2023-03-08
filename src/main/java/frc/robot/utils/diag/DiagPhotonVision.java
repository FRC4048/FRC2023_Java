package frc.robot.utils.diag;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Base class for Diagnosable that have an initial value (their "current" value) and that are required to change their
 * value by a certain amount from that
 */
public abstract class DiagPhotonVision implements Diagnosable {

    private final String name;
    private final String title;
    private int firstTagId;
    private double firstTimestamp;
    protected GenericEntry networkTableEntry;


    public DiagPhotonVision(String title, String name) {
        this.title = title;
        this.name = name;
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
        firstTagId = -1;
        firstTimestamp = -1;
    }

    boolean getDiagResult() {
        if (firstTagId == -1 || firstTimestamp == -1){
            firstTagId = getTagId();
            firstTimestamp = getTagTimestamp();
        } else return getTagTimestamp()==firstTimestamp;
        return false;
    }

    protected abstract int getTagId();
    protected abstract double getTagTimestamp();
}
