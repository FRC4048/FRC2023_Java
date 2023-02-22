package frc.robot.utils.diag;

import com.ctre.phoenix.sensors.WPI_CANCoder;

/**
 * A diagnostics class for digital encoder. The diagnostics will turn green once the encoder has traveled at least a given
 * distance from its initial position (measured at initialization or after a reset)
 */
public class DiagSparkMaxAbsEncoder extends DiagDistanceTraveled {

    private WPI_CANCoder canCoder;

    /**
     * Constructor
     *
     * @param name            - the name of the unit. Will be used on the Shuffleboard
     * @param requiredTravel  - the required difference between the initial position to qualify for success
     * @param canSparkMax     - the encoder instance to test
     */
    public DiagSparkMaxAbsEncoder(String title, String name, double requiredTravel, WPI_CANCoder canCoder) {
        super(title, name, requiredTravel);
        this.canCoder = canCoder;
        reset();
    }

    @Override
    protected double getCurrentValue() {
        return canCoder.getAbsolutePosition();
    }
}
