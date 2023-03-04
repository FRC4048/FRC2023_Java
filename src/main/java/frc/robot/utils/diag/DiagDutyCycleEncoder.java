package frc.robot.utils.diag;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * A diagnostics class for digital encoder. The diagnostics will turn green once the encoder has traveled at least a given
 * distance from its initial position (measured at initialization or after a reset)
 */
public class DiagDutyCycleEncoder extends DiagDistanceTraveled {

    private DutyCycleEncoder encoder;

    /**
     * Constructor
     *
     * @param name            - the name of the unit. Will be used on the Shuffleboard
     * @param requiredTravel  - the required difference between the initial position to qualify for success
     * @param encoder         - the encoder instance to test
     */
    public DiagDutyCycleEncoder(String title, String name, double requiredTravel, DutyCycleEncoder encoder) {
        super(title, name, requiredTravel);
        this.encoder = encoder;
        reset();
    }

    @Override
    protected double getCurrentValue() {
        return encoder.get();
    }
}
