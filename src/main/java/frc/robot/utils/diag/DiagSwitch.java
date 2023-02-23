package frc.robot.utils.diag;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Diagnostics class for digital switch; it is a DiagBoolean object.
 */
public class DiagSwitch extends DiagBoolean {

    private DigitalInput digitalInput;

    /**
     * Constructor
     *
     * @param name            the name of the unit. Will be used on the Shuffleboard
     * @param digitalInput    - the DigitalInput the switch is connected to
     */
    public DiagSwitch(String title, String name, DigitalInput digitalInput) {
        super(title, name);
        this.digitalInput = digitalInput;
    }

    @Override
    protected boolean getValue() {
        return digitalInput.get();
    }
}
