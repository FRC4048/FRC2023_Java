package frc.robot.utils;

public enum GripperPos {
    OPEN(1),
    CLOSED_ON_CUBE(.3),
    CLOSED_ON_CONE(.5),
    STOP_CLOSED(0);

    public final double pos;

    GripperPos(double pos) {
        this.pos = pos;
    }
}
