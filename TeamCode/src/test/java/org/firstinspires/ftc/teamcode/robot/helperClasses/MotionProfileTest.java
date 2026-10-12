package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MotionProfileTest {
    @Test
    public void trapezoidFromRest() {
        // accel 0.5 s (12.5), cruise 1.5 s (75), decel 0.5 s (12.5)
        MotionProfile p = new MotionProfile(0, 100, 0, 50, 100);
        assertEquals(2.5, p.getTotalTime(), 1e-9);
        assertEquals(50, p.get(1.0).velocity, 1e-9);
        assertEquals(100, p.get(p.getTotalTime()).position, 1e-9);
        assertEquals(0, p.get(p.getTotalTime()).velocity, 1e-9);
    }

    @Test
    public void positionIsContinuousAndMonotone() {
        MotionProfile p = new MotionProfile(0, 100, 20, 50, 100);
        double prev = p.get(0).position;
        for (double t = 0.001; t < p.getTotalTime(); t += 0.001) {
            double x = p.get(t).position;
            assertTrue("went backwards at t=" + t, x >= prev - 1e-9);
            assertTrue("jumped at t=" + t, x - prev < 0.1);
            prev = x;
        }
        assertEquals(100, prev, 0.1);
    }

    @Test
    public void shortMoveBecomesTriangleAndStillArrives() {
        MotionProfile p = new MotionProfile(10, 0, -5, 100, 20);
        assertEquals(0, p.get(p.getTotalTime() - 1e-9).position, 1e-3);
        assertTrue(p.get(p.getTotalTime() / 2).velocity < 0); // moving toward the goal (negative)
    }

    @Test
    public void reusesStateObject() {
        MotionProfile p = new MotionProfile(0, 10, 0, 5, 5);
        assertTrue(p.get(0.1) == p.get(0.2)); // no allocation per call
    }
}
