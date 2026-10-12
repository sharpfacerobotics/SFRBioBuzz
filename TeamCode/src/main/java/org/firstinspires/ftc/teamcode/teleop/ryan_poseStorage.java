package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    // static -> survives from auto into teleOp (as long as the robot app isn't restarted)
    public static Pose currentPose = null;

    // call in auto with follower.getPose()
    // best: call it EVERY loop in auto, so the pose is saved even if the 30s timer cuts auto off
    public void updatePoseStoragePose(Pose currPose) {
        currentPose = currPose;
    }

    // call in teleOp init, then pass into follower.setStartingPose(...)
    public Pose getPoseStoragePose() {
        return currentPose;
    }

    // check this before using the pose in teleOp (null = auto never ran)
    public boolean hasPose() {
        return currentPose != null;
    }

    // optional: call at the end of teleOp so an old pose doesn't carry into the next match
    public void clearPoseStoragePose() {
        currentPose = null;
    }
}
