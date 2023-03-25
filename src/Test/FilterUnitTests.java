package frc.robot.apriltags;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.beans.Transient;

import edu.wpi.first.util.CircularBuffer;

public class FilterUnitTests {
    @Test
    public void testBasicFilter() {
        PoseFilter filter = new PoseFilter(3, 5);

       double result = filter.calculate(50);
       assertEquals(50, result, .001);

       double second = filter.calculate(40);
       double average = filter.getAverage();
       assertEquals(40, second, .001);
       assertEquals(45.0, average, .001);

       double third = filter.calculate(45);
       assertEquals(45, third, .001);
       average = filter.getAverage();
       assertEquals(45, average, .001);

       double fourth = filter.calculate(48);
       assertEquals(48, fourth, .001);

       average = filter.getAverage();
       assertEquals(44.33333, average, .0001);
    }

    @Test
    public void testCircularBuffer() {
        CircularBuffer buffer = new CircularBuffer(3);
        buffer.addLast(1.0);
        buffer.addLast(2.0);

        assertEquals(1.0, buffer.get(0), .001);
        assertEquals(2.0, buffer.get(1), .001);
        assertEquals(0.0, buffer.get(2), .001);
    }

    @Test 
    public void testIsValid() {
        PoseFilter filter = new PoseFilter(3, 50);
        filter.calculate(50);
        filter.calculate(50);
        filter.calculate(50);

        double avg = filter.getAverage();

        boolean isValid = filter.isValid(avg, 101, 50);
        assertFalse(isValid);

        assertTrue(filter.isValid(avg, 99, 50));
        assertTrue(filter.isValid(avg, 100, 50));

        assertTrue(filter.isValid(avg, 1, 50));
        assertTrue(filter.isValid(avg, 0, 50));
        assertFalse(filter.isValid(avg, -1, 50));
    }

    @Test
    public void testNegativeFilter() {
        PoseFilter filter = new PoseFilter(3, 10);
        filter.calculate(10);
        filter.calculate(0);
        filter.calculate(-1);
        filter.calculate(-1);
        filter.calculate(-1);
        filter.calculate(0);
        filter.calculate(0);
        filter.calculate(0);
        filter.calculate(-10);
        filter.calculate(-100);
        filter.calculate(0);
        filter.calculate(-10);
        filter.calculate(-10);
        filter.calculate(-10);

        double avg = filter.getAverage();
        System.out.println(avg);
        assertEquals(-10, avg, 0.0001);
    }

    @Test
    public void testFilterInput() {
        PoseFilter filter = new PoseFilter(3, 10);
        filter.calculate(10);
        filter.calculate(0);
        filter.calculate(-1);
        filter.calculate(-1);
        filter.calculate(-1);
        filter.calculate(0);
        filter.calculate(0);
        filter.calculate(0);
        filter.calculate(1);
        filter.calculate(2);
        filter.calculate(3);
        double[] input = filter.getValuesInFilter();
        assertEquals(1, input[1], 0.01);
        assertEquals(2, input[2], 0.01);
        assertEquals(3, input[3], 0.01);
    }
}