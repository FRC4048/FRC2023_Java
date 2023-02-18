package frc.robot.utils;
import frc.robot.subsystems.ExtenderSubsystem;

public enum ExtenderPosition {
    BOTTOM_ROW(10000), 
    MIDDLE_ROW(20000), 
    TOP_ROW(30000), 
    RETRACT_FULL(0), 
    EXTEND_FULL(40000);

    public double position;

    ExtenderPosition(double position) {
        this.position = position;
    }
    
    public double getPosition() {
        return position;
    }

}
