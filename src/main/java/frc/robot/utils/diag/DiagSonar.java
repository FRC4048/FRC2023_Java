package frc.robot.utils.diag;

import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * Diagnostics class for the Sonar. It is a DiagMinMax object.
 * Give it a max and minimum distance to test.
 */
public class DiagSonar extends DiagMinMax {

    private Ultrasonic sonar;

    /**
     * Constructor
     * 
     * @param name                    -Name of the sensor, to be used on Shuffleboard
     * @param sonar                   -The Ultrasonic object to be tested.
     * @param minDistance             -The minimum testing distance for the sonar; the one that will be tested against.
     * @param maxDistance             -The maximum testing distance for the sonar; the one that will be tested against.
     */
    public DiagSonar(String name, Ultrasonic sonar, double minDistance, double maxDistance){
        super(name, minDistance, maxDistance);
        this.sonar = sonar;
    }

    @Override
    double getSensorReading() {
        return sonar.getRangeInches();
    }
}
