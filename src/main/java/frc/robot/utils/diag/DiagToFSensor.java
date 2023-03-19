package frc.robot.utils.diag;

import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * Diagnostics class for the Sonar. It is a DiagMinMax object.
 * Give it a max and minimum distance to test.
 */
public class DiagToFSensor extends DiagMinMax {

    private Rev2mDistanceSensor sensor;

    /**
     * Constructor
     * 
     * @param name                    -Name of the sensor, to be used on Shuffleboard
     * @param sonar                   -The Ultrasonic object to be tested.
     * @param minDistance             -The minimum testing distance for the sonar; the one that will be tested against.
     * @param maxDistance             -The maximum testing distance for the sonar; the one that will be tested against.
     */
    public DiagToFSensor(String title, String name, Rev2mDistanceSensor sensor, double minDistance, double maxDistance){
        super(title, name, minDistance, maxDistance);
        this.sensor = sensor;
    }

    @Override
    double getSensorReading() {
        return sensor.getRange();
    }
}
