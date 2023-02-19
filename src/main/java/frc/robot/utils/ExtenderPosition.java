package frc.robot.utils;
import frc.robot.Constants;

public enum ExtenderPosition {
    BOTTOM_ROW(Constants.EXTENDER_BOTTOM_ROW), 
    MIDDLE_ROW(Constants.EXTENDER_MIDDLE_ROW), 
    TOP_ROW(Constants.EXTENDER_TOP_ROW), 
    RETRACT_FULL(Constants.EXTENDER_RETRACTFULL);

    public double position;

    ExtenderPosition(double position) {
        this.position = position;
    }
    
    public double getPosition() {
        return position;
    }

}
