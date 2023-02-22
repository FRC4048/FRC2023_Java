package frc.robot.utils.diag;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Diagnostics class for digital switch connected directly to the talon SRX
 * it is a DiagBoolean subclass.
 */
public class DiagTalonSrxSwitch extends DiagBoolean {
    public DiagTalonSrxSwitch(String title, String name) {
        super(title, name);
    }

    public enum Direction {FORWARD, REVERSE};
    private WPI_TalonSRX talonSRX;
    private Direction direction;

    /*
     * Constructor
     *
     * @param name      -the name of the unit. Will be used on the Shuffleboard
     * @param talonSRX  -the talon SRX to read the switch value from
     */

    public DiagTalonSrxSwitch(String title, String name, WPI_TalonSRX talonSRX, Direction direction) {
        super(title, name);
        this.talonSRX = talonSRX;
        this.direction = direction;
    }
    @Override
    protected boolean getValue() {
        switch (direction) {
            case FORWARD:
                return talonSRX.getSensorCollection().isFwdLimitSwitchClosed();
            case REVERSE:
                return talonSRX.getSensorCollection().isRevLimitSwitchClosed();
            default:
                return false;
        }
    }

}
