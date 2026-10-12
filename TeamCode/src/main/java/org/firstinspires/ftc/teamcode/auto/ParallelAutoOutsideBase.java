package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * "Parallel" auto, OUTSIDE start (audience wall). Owner: Vivek.
 *
 * <p>Pairs with the INSIDE auto: this one works the audience end of our half - the GARDEN's 4
 * POLLEN and the FLOWER on our wall - shooting from the low spot, then parks.</p>
 *
 * <p>Poses are RED (see FieldPoses); blue is rotated automatically.</p>
 */
public abstract class ParallelAutoOutsideBase extends AutoBase {
    @Override
    protected Pose startPose() {
        return FieldPoses.START_AUDIENCE_WALL;
    }

    @Override
    protected void buildRoutine() {
        drive("to low shoot spot", FieldPoses.SHOOT_LOW);
        shoot("pre-loads");

        drive("line up garden", FieldPoses.GARDEN_LINE_UP);
        driveIntaking("sweep garden pollen", FieldPoses.GARDEN_END);
        drive("back to low shoot spot", FieldPoses.SHOOT_LOW);
        shoot("garden pollen");

        drive("line up wall flower", FieldPoses.WALL_FLOWER_LINE_UP);
        driveIntaking("take wall flower pollen", FieldPoses.WALL_FLOWER);
        drive("back to low shoot spot", FieldPoses.SHOOT_LOW);
        shoot("wall flower pollen");

        drive("park in loading zone", FieldPoses.WALL_FLOWER_LINE_UP, FieldPoses.PARK);
    }
}
