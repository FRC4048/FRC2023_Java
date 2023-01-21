package frc.robot.utils.limelight;

public class CameraDistance {
    private double forward;
    private double sideways;

    public CameraDistance(double forward, double sideways) {
        this.forward = forward;
        this.sideways = sideways;
    }

    public double getForward() {
        return forward;
    }

    public double getSideways() {
        return sideways;
    }
}
