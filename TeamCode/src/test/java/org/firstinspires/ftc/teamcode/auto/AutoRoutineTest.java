package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;
import org.junit.Test;

/** BIOBUZZ auto rules checked on every auto, both alliances. */
public class AutoRoutineTest {
    private static final double ROBOT_HALF = 9.0; // ~18 in robot

    private static AutoBase[] allAutos() {
        return new AutoBase[]{
                new SubordinateAutoRed(), new SubordinateAutoBlue(),
                new ParallelAutoInsideRed(), new ParallelAutoInsideBlue(),
                new ParallelAutoOutsideRed(), new ParallelAutoOutsideBlue()};
    }

    @Test
    public void blueIsTheRotationOfRed() {
        Pose red = new Pose(30, 40, Math.toRadians(30));
        Pose blue = new SubordinateAutoBlue().field(red);
        assertEquals(144 - 30, blue.x(), 1e-9);
        assertEquals(144 - 40, blue.y(), 1e-9);
        // Pedro stores heading in [0, 2pi): -150 deg comes back as 210 deg
        assertEquals(0, Math.sin(blue.heading() - Math.toRadians(-150)), 1e-9);
        assertEquals(1, Math.cos(blue.heading() - Math.toRadians(-150)), 1e-9);
        assertEquals(30, new SubordinateAutoRed().field(red).x(), 1e-9);
    }

    @Test
    public void everyAutoStaysOnItsOwnHalf() { // G402
        for (AutoBase auto : allAutos()) {
            auto.prepareRoutine();
            assertEquals(auto.getClass().getSimpleName(), 0, auto.getSideViolations());
            assertTrue(auto.getStepCount() >= 3);
        }
    }

    @Test
    public void everyAutoStartsLegally() { // G304: touching our wall, not in the loading zone
        for (AutoBase auto : allAutos()) {
            Pose start = auto.prepareRoutine();
            boolean red = auto.getClass().getSimpleName().endsWith("Red");
            Pose r = red ? start : new Pose(144 - start.x(), 144 - start.y(), 0); // compare in red frame
            double wallGap = Math.min(Math.min(r.x(), r.y()), 144 - r.y());
            assertEquals(auto.getClass().getSimpleName() + " not touching a wall", ROBOT_HALF, wallGap, 0.5);
            assertFalse(auto.getClass().getSimpleName() + " starts in loading zone", inLoadingZone(r));
        }
    }

    @Test
    public void everyAutoParksWithoutTouchingTheWall() { // PARK 5 + keep LEAVE 3
        for (AutoBase auto : allAutos()) {
            auto.prepareRoutine();
            Pose end = auto.getEndPose();
            boolean red = auto.getClass().getSimpleName().endsWith("Red");
            Pose r = red ? end : new Pose(144 - end.x(), 144 - end.y(), 0);
            assertTrue(auto.getClass().getSimpleName() + " not parked", inLoadingZone(r));
            assertTrue(auto.getClass().getSimpleName() + " touching wall", r.x() - ROBOT_HALF > 1.0);
        }
    }

    /** Robot footprint overlaps the red loading zone (x 0..11, y 96..119). */
    private static boolean inLoadingZone(Pose redFramePose) {
        double zx = FieldGeometry.RED_LOADING_ZONE_X, zy = FieldGeometry.RED_LOADING_ZONE_Y;
        return redFramePose.x() - ROBOT_HALF < zx + 5.5 && Math.abs(redFramePose.y() - zy) < 11.5 + ROBOT_HALF;
    }
}
