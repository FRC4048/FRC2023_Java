package frc.robot.utils.diag;

import com.revrobotics.CANSparkMax;

/**
 * A diagnostics class for digital encoder. The diagnostics will turn green once the encoder has traveled at least a given
 * distance from its initial position (measured at initialization or after a reset)
 */
public class DiagSparkMaxEncoder extends DiagDistanceTraveled {

    private CANSparkMax canSparkMax;

    /**
     * Constructor
     *
     * @param name            - the name of the unit. Will be used on the Shuffleboard
     * @param requiredTravel  - the required difference between the initial position to qualify for success
     * @param canSparkMax     - the encoder instance to test
     */
    public DiagSparkMaxEncoder(String title, String name, double requiredTravel, CANSparkMax canSparkMax) {
        super(title, name, requiredTravel);
        this.canSparkMax = canSparkMax;
        reset();
    }

    @Override
    protected double getCurrentValue() {
        return canSparkMax.getEncoder().getPosition();
    }
}
