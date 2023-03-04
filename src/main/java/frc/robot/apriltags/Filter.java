package frc.robot.apriltags;

import edu.wpi.first.util.CircularBuffer;

public class Filter {

    private int counter = 0;
    private double average = 1;
    private int maxInputs = 1;
    private CircularBuffer values;
    private double tolerance;

    /**
     * Creates the filter
     * 
     * @param maxInputs The number of inputs taken by before the filter starts
     *                  filtering.
     * @param tolerence The size of the range that the value can deviate from. ex.
     *                  range of 5 will allow new inputs to be 5 less than and 5
     *                  greater than the average
     */
    public Filter(int maxInputs, double tolerance) {
        this.maxInputs = maxInputs;
        this.tolerance = tolerance;
        values = new CircularBuffer(maxInputs);
    }

    /**
     * ONLY TO BE USED FOR UNIT TESTS
     * 
     * @param averageLast average of the past few values
     * @param input       new value to check if valid or not
     * @param tolerance   The size of the range that the value can deviate from. ex.
     *                    range of 5 will allow new inputs to be 5 less than and 5
     *                    greater than the average
     * 
     * @return if the value is valid or not
     */
    public boolean isValid(double averageLast, double input, double tolerance) {
        return (input <= (averageLast + tolerance) && input >= (averageLast - tolerance));
    }

    /**
     * Calculates the new value from the previous values and/or adds a new value to
     * compare future values with.
     * 
     * @param newInput input to be calculated by and/or added to the filter
     * 
     * @return new calculated values
     */
    public double calculate(double newInput) {
        if (!isValid(average, newInput, tolerance) && counter >= maxInputs) {
            newInput = values.getLast();
        } else {
            values.addLast(newInput);
        }
        double intermediateAverage = 0;
        for (int temp = values.size(); temp > 0; temp--) {
            intermediateAverage = intermediateAverage + values.get(temp);
        }
        intermediateAverage = intermediateAverage / values.size();
        average = intermediateAverage;
        counter++;
        return newInput;
    }

    /**
     * Resets the filter so that it will ignore all previous values given and will
     * gain new filter
     */
    public void resetFilter() {
        counter = 0;
        values.clear();
    }

    /**
     * @return The current average of all previous valid values
     */
    public double getAverage() {
        return average;
    }
}
