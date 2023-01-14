package frc.robot.utils.diag;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Diagnostic class for the Optical sensor; it is a DiagBoolean object.
 */
public class DiagOpticalSensor extends DiagBoolean {

    private DigitalInput digitalInput;

    /**
     * Constructor
     * 
     * @param name            - The sensor's name, which will be shown on Shuffleboard
     * @param digitalInput    - The DigitalInput pin the sensor is connected to
     */
    public DiagOpticalSensor(String name, DigitalInput digitalInput){
        super(name);
        this.digitalInput = digitalInput;
    }

    @Override
    protected boolean getValue() {
        return digitalInput.get();
    }
}
