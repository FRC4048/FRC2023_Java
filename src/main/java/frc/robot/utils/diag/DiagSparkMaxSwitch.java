package frc.robot.utils.diag;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch.Type;

/**
 * Diagnostics class for digital switch connected directly to the talon SRX
 * it is a DiagBoolean subclass.
 */
public class DiagSparkMaxSwitch extends DiagBoolean {
    public DiagSparkMaxSwitch(String title, String name) {
        super(title, name);
    }

    public enum Direction {FORWARD, REVERSE};
    private CANSparkMax canSparkMax;
    private Direction direction;

    /*
     * Constructor
     *
     * @param name      -the name of the unit. Will be used on the Shuffleboard
     * @param talonSRX  -the talon SRX to read the switch value from
     */

    public DiagSparkMaxSwitch(String title, String name, CANSparkMax canSparkMax, Direction direction) {
        super(title, name);
        this.canSparkMax = canSparkMax;
        this.direction = direction;
    }
    @Override
    protected boolean getValue() {
        switch (direction) {
            case FORWARD:
                return canSparkMax.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
            case REVERSE:
                return canSparkMax.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
            default:
                return false;
        }
    }

}
