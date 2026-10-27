package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MotionProfileTest {
    private static void checkProfile(double start, double goal, double v0, double maxV, double maxA) {
        MotionProfile profile = new MotionProfile(start, goal, v0, maxV, maxA);
        double total = 0;
        while (!profile.isFinished(total)) {
            total += 0.0005;
        }
        MotionProfile.State end = profile.get(total - 1e-7);
        assertEquals("ends at goal", goal, end.position, 1e-3);
        assertEquals("ends stopped", 0.0, end.velocity, 1e-3);
        // Integrating velocity must land on the goal, and speed must never pass maxV
        double x = start;
        double dt = total / 20000;
        for (int i = 0; i < 20000; i++) {
            double t = i * dt;
            double v = profile.get(t).velocity;
            assertTrue(Math.abs(v) <= maxV + 1e-9);
            x += (v + profile.get(t + dt).velocity) / 2 * dt;
        }
        assertEquals("velocity integrates to goal", goal, x, 1e-2);
    }

    @Test public void trapezoidFromRest() { checkProfile(0, 100, 0, 30, 20); }
    @Test public void triangleShortMove() { checkProfile(0, 10, 0, 30, 20); }
    @Test public void startsAlreadyMoving() { checkProfile(0, 100, 10, 30, 20); }
    @Test public void movesBackwards() { checkProfile(50, -20, -5, 30, 20); }
    @Test public void shortMoveWhileMoving() { checkProfile(0, 5, 10, 30, 20); }

    @Test public void zeroDistanceIsDoneImmediately() {
        MotionProfile profile = new MotionProfile(3, 3, 0, 30, 20);
        assertTrue(profile.isFinished(0));
        assertEquals(3, profile.get(0).position, 1e-12);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonPositiveLimits() { new MotionProfile(0, 1, 0, 0, 20); }
}
