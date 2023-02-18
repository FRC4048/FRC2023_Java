package frc.robot.utils;
import frc.robot.subsystems.ExtenderSubsystem;

public enum ExtenderPosition {
    BOTTOM_ROW(500), 
    MIDDLE_ROW(1000), 
    TOP_ROW(1500), 
    RETRACT_FULL(0), 
    EXTEND_FULL(2000);

    public double position;

    ExtenderPosition(double position) {
        this.position = position;
    }
    
    public double getPosition() {
        return position;
    }
}
