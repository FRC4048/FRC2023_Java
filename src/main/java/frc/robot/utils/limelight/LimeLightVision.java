package frc.robot.utils.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightVision {
    public static final int LED_ON = 3;
    public static final int LED_BLINK = 2;
    public static final int LED_OFF = 1;

    /** The relative height of the target to the camera, for trig calculations */
    private double relativeHeight;
    private double cameraAngle;

    NetworkTable table;
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry ts;
    NetworkTableEntry tl;
    NetworkTableEntry stream;

    /**
     * Utility class to calculate distances based off of LimeLIght camera angle calculation.
     * @param cameraHeight height of Limelight camera, in inches
     * @param targetHeight height of field target, in inches
     * @param cameraAngle camera mounting angle, in degrees, where 0 is horizontal and down is negative
     */
    public LimeLightVision(double cameraHeight, double targetHeight, double cameraAngle) {
        this.relativeHeight = targetHeight - cameraHeight;
        this.cameraAngle = cameraAngle;
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tl = table.getEntry("tl");
        stream = table.getEntry("stream");
    }

    // This method is required in order to allow tests to be written since NetworkTable is final and cannot be mocked
    LimeLightVision(boolean DO_NOT_USE_FOR_TESTING_ONLY, double cameraHeight, double targetHeight, double cameraAngle) {
        this.relativeHeight = targetHeight - cameraHeight;
        this.cameraAngle = cameraAngle;
    }

    /**
     * Calculate and return the distances from the camera.
     * The calculated distances are in inches, as calculated by the triangulation of the vertical distance between the
     * camera and target and the detected angles.
     * forward is the "straight" distance from the camera to the target
     * sideways is the "lateral" distance from the camera to the target
     * @return Camera distance calculated from the limelight, null if no target acquired
     */
    public CameraDistance getTargetDistance() {
        double validTarget = tv.getDouble(0.0);
        if ( validTarget != 1.0 ) {
            return null;
    }

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);

        return calcCameraDistance(x, y);
    }

    /**
     * Return true if the limelight has a target
     * 
     * @return true if the camera has a target
     */
    public boolean hasTarget() {
        return (tv.getDouble(0.0) > 0);
    }

    /**
     * Calculate and return the camera detected angles
     * The angles are returned by the Limelight camera and are in degrees.
     * tx is the horizontal angle between the target center and the camera crosshairs
     * ty is the vertical angle between the target center and the camera crosshairs
     * @return the X and Y angles detected by the camera, Null in case no target is acquired
     */
    public CameraAngles getCameraAngles() {
        double validTarget = tv.getDouble(0.0);
        if ( validTarget != 1.0 ) {
            return null;
        }

        return new CameraAngles(tx.getDouble(0.0), ty.getDouble(0.0));
    }

    public void setLedOn() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LED_ON);
    }

    public void setLedOff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LED_OFF);
    }

    public void setLedBlink() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LED_BLINK);
    }
    
    public void setPipeline(int pipelineMode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineMode);
    }

    /**
     * The Limelight camera has 3 streaming modes if a second USB camera is plugged in.
     * 0 = both, side by side
     * 1 = Limelight stream with USB camera in the corner
     * 2 = USB camera stream with Limelight in the corner
     */
    public void setStream(double option) {
        if (option >= 0.0 && option <= 2.0)
        {
            stream.setDouble(option);
        }
    }

    public double getStream() {
        return stream.getDouble(0.0);
    }

    /**
     * Internal utility to calculate the distances, in inches, from the camera based off of the viewed angles
     */
    private CameraDistance calcCameraDistance(double angleX, double angleY) {
        double forwardDistance = (relativeHeight) / Math.tan(Math.toRadians(cameraAngle + angleY));
        double sidewaysDistance = forwardDistance * Math.tan(Math.toRadians(angleX));

        return new CameraDistance(forwardDistance, sidewaysDistance);
    } 

    public double calcDirectDistanceToTarget (double angleY){
        double h = (relativeHeight)/Math.sin(Math.toRadians(cameraAngle+angleY));
        return h;
    } 

    public double calcHorizontalDistanceToTarget (double angleY){
		/*Assumes y offset from camera is in degrees*/
        //+10 Accounts for distance from camera to bumper and distance to center of hoop
        double horizontal = (relativeHeight)/Math.tan(Math.toRadians(cameraAngle+angleY)) + 10;  
		return horizontal;
    } 
    
}
