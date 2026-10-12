package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretMathTest {
    private static double deg(double d) {
        return Math.toRadians(d);
    }

    @Test
    public void plainTargetInsideLimits() {
        assertEquals(deg(45), Turret.chooseReachableTarget(deg(45), 0), 1e-9);
    }

    @Test
    public void wrapsTheShortWayWhenThatStaysInsideLimits() {
        // at +170, target -175 is the same as +185, which is inside the 190 limit
        assertEquals(deg(185), Turret.chooseReachableTarget(deg(-175), deg(170)), 1e-9);
    }

    @Test
    public void unwindsWhenTheShortWayWouldPassALimit() {
        // +200 would be past the limit, so go the long way round to -160
        assertEquals(deg(-160), Turret.chooseReachableTarget(deg(-160), deg(170)), 1e-9);
    }
}
