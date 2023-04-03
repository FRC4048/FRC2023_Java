package frc.robot.apriltags;

import java.util.Arrays;

import edu.wpi.first.util.CircularBuffer;

public class PoseFilter {

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
    public PoseFilter(int maxInputs, double tolerance) {
        this.maxInputs = maxInputs;
        this.tolerance = tolerance;
        values = new CircularBuffer(maxInputs);
    }

    /**
     * Checks if the inputed value should be filtered or not using the tolerence given and the average of the last few values
     * 
     * @param average average of the past few values
     * @param input       new value to check if valid or not
     * @param tolerance   The size of the range that the value can deviate from. ex.
     *                    range of 5 will allow new inputs to be 5 less than and 5
     *                    greater than the average
     * @param median the median
     * 
     * @return if the value is valid or not
     */
    public boolean isValid(double average, double input, double tolerance, double median) {
        if((average + tolerance) < median || (average - tolerance) > median) {
            return(input <= (median + tolerance));
        }
        else {
            return (input <= (average + tolerance) && input >= (average - tolerance));
        }
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
        if (counter >= maxInputs && !isValid(average, newInput, tolerance, getMedian())) {
            newInput = values.getLast();
        } else {
            values.addLast(newInput);
        }
        double intermediateAverage = 0;
        for (int i = values.size(); i > 0; i--) {
            intermediateAverage = intermediateAverage + values.get(i);
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

    /**
     * @return The last values used when doing calculations in the order {oldest value, 2nd oldest value, ... 2nd newest value, newest value}
     */
    public double[] getValuesInFilter() {
        double[] output = new double[values.size() - 1];
        for (int i = 0; i >= (values.size() - 1); i++) {
            output[i] = values.get(i);
        }
        return output;
    }

    /**
     * @return The median of the filtered values
     */
    public double getMedian() {
        double[] listOfValues = getValuesInFilter();
        Arrays.sort(listOfValues);
        if ((values.size() % 2) == 1) {
            return ((listOfValues[values.size()/2] + listOfValues[values.size()/2])/2);
        }
        else {
            return listOfValues[values.size()/2];
        }
    }
}