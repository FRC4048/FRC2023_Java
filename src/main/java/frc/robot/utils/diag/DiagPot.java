package frc.robot.utils.diag;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * A diagnostics class for analog potentiometer. It is a DiagMinMax object.
 * Give it the maximum and minimum voltages for testing with.
 */
public class DiagPot extends DiagMinMax {


    private AnalogPotentiometer pot;


    /**
     * Constructor
     *
     * @param name            - the name of the unit. Will be used on the Shuffleboard
     * @param minVoltage      - the minimum value the pot needs to hit to qualify for success
     * @param maxVoltage      - the maximum value the pot needs to hit to qualify for success
     * @param pot             - the pot instance to test
     */
    public DiagPot(String title, String name, double minVoltage, double maxVoltage, AnalogPotentiometer pot) {
        super(title, name, minVoltage, maxVoltage);
        this.pot = pot;
    }

    @Override
    double getSensorReading(){
        return pot.get();
    }
}