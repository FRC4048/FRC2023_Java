package frc.robot.utils.diag;

import edu.wpi.first.wpilibj.Encoder;

/**
 * A diagnostics class for digital encoder. The diagnostics will turn green once the encoder has traveled at least a given
 * distance from its initial position (measured at initialization or after a reset)
 */
public class DiagEncoder extends DiagDistanceTraveled {

    private Encoder encoder;

    /**
     * Constructor
     *
     * @param name            - the name of the unit. Will be used on the Shuffleboard
     * @param requiredTravel  - the required difference between the initial position to qualify for success
     * @param encoder         - the encoder instance to test
     */
    public DiagEncoder(String name, int requiredTravel, Encoder encoder) {
        super(name, requiredTravel);
        this.encoder = encoder;
        reset();
    }

    @Override
    protected int getCurrentValue() {
        return encoder.get();
    }
}
