package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * "Parallel" auto, INSIDE start (red wall, near the hive). Owner: Vivek.
 *
 * <p>Runs full cycles alongside a partner running the OUTSIDE auto - they split the pickups
 * (this one takes the far FLOWER, outside takes the GARDEN and wall FLOWER) so they never
 * collide. Pre-loads, far flower, then park.</p>
 *
 * <p>Poses are RED (see FieldPoses); blue is rotated automatically.</p>
 */
public abstract class ParallelAutoInsideBase extends AutoBase {
    @Override
    protected Pose startPose() {
        return FieldPoses.START_RED_WALL;
    }

    @Override
    protected void buildRoutine() {
        drive("to shoot spot", FieldPoses.SHOOT_CENTER);
        shoot("pre-loads");

        drive("line up far flower", FieldPoses.FAR_FLOWER_LINE_UP);
        driveIntaking("take far flower pollen", FieldPoses.FAR_FLOWER);
        drive("back to shoot spot", FieldPoses.SHOOT_CENTER);
        shoot("far flower pollen");

        drive("park in loading zone", FieldPoses.PARK);
    }
}
