package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class InterpLUTTest {
    private final InterpLUT lut = new InterpLUT(new double[]{0, 10, 20, 30}, new double[]{0, 5, 20, 22});

    @Test
    public void passesThroughControlPoints() {
        assertEquals(0, lut.get(0), 1e-9);
        assertEquals(5, lut.get(10), 1e-9);
        assertEquals(20, lut.get(20), 1e-9);
        assertEquals(22, lut.get(30), 1e-9);
    }

    @Test
    public void clampsOutsideDomain() {
        assertEquals(0, lut.get(-50), 1e-9);
        assertEquals(22, lut.get(500), 1e-9);
    }

    @Test
    public void staysMonotoneBetweenMonotonePoints() {
        double prev = lut.get(0);
        for (double x = 0.1; x <= 30; x += 0.1) {
            double y = lut.get(x);
            assertTrue("dropped at x=" + x, y >= prev - 1e-9);
            prev = y;
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsTooFewPoints() {
        new InterpLUT(new double[]{1}, new double[]{1});
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonIncreasingX() {
        new InterpLUT(new double[]{0, 5, 5}, new double[]{0, 1, 2});
    }
}
