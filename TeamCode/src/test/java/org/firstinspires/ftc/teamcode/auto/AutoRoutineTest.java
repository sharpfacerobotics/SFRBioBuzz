package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class AutoRoutineTest {
    @Test
    public void redMirrorsBlue() {
        Pose blue = new Pose(30, 40, Math.toRadians(30));
        Pose red = new SubordinateAutoRed().field(blue);
        assertEquals(144 - 30, red.x(), 1e-9);
        assertEquals(40, red.y(), 1e-9);
        assertEquals(Math.toRadians(150), red.heading(), 1e-9);
        Pose same = new SubordinateAutoBlue().field(blue);
        assertEquals(30, same.x(), 1e-9);
    }
}
