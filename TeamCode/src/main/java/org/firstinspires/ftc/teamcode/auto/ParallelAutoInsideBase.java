package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * "Parallel" auto from the INSIDE start (near our hive). Owner: Vivek.
 *
 * <p>Runs full cycles in parallel with our partner: preloads, then the close and middle
 * spike marks, shooting from the close zone each time, then parks.</p>
 *
 * <p>Poses are for BLUE in Pedro coordinates (inches, degrees). Red is mirrored automatically.
 * TODO(measure): every pose below is a placeholder until the BioBuzz field is measured.</p>
 */
public abstract class ParallelAutoInsideBase extends AutoBase {
    protected Pose startPose        = pose(24, 120, 315);
    protected Pose shootPose        = pose(48, 96, 180);
    protected Pose closeSpikeLineUp = pose(48, 84, 180);
    protected Pose closeSpikeEnd    = pose(18, 84, 180);
    protected Pose midSpikeLineUp   = pose(48, 60, 180);
    protected Pose midSpikeEnd      = pose(18, 60, 180);
    protected Pose midReturnControl = pose(44, 66, 180);
    protected Pose parkPose         = pose(40, 72, 180);

    @Override
    protected Pose startPose() {
        return startPose;
    }

    @Override
    protected void buildRoutine() {
        drive("to shoot zone", shootPose);
        shoot("preloads");

        drive("line up close spike", closeSpikeLineUp);
        driveIntaking("intake close spike", closeSpikeEnd);
        drive("back to shoot zone", shootPose);
        shoot("close spike");

        drive("line up middle spike", midSpikeLineUp);
        driveIntaking("intake middle spike", midSpikeEnd);
        drive("back to shoot zone", midReturnControl, shootPose);
        shoot("middle spike");

        drive("park", parkPose);
    }
}
