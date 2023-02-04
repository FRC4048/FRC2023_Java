package frc.robot.utils;
import frc.robot.subsystems.ExtenderSubsystem;

public enum ExtenderPosition {
    BOTTOM_ROW(50), 
    MIDDLE_ROW(5050), 
    TOP_ROW(10100), 
    RETRACT_FULL(0), 
    EXTEND_FULL(10500);

    public double position;

    ExtenderPosition(double position) {
        this.position = position;
    }
    
    public double getPosition() {
        return position;
    }
}
