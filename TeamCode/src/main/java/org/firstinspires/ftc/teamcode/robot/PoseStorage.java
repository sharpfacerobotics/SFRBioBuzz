package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's pose from auto to TeleOp. Owner: Ryan.
 *
 * <p>Static, so it survives from auto into TeleOp as long as the robot app isn't restarted.
 * This is the one subsystem without the RobotHardware constructor - it holds no hardware and
 * has to outlive any single OpMode.</p>
 */
public class PoseStorage {
    private static Pose currentPose = null;

    private PoseStorage() {
    }

    /**
     * Call in auto with follower.pose(). Best: call it EVERY loop in auto, so the pose is
     * saved even if the 30s timer cuts auto off. (Pose is immutable, so storing it is safe.)
     */
    public static void save(Pose pose) {
        currentPose = pose;
    }

    /** Call in TeleOp init, then pass into follower.setPose(...). Null if auto never ran. */
    public static Pose get() {
        return currentPose;
    }

    /** Check this before using the pose in TeleOp (false = auto never ran). */
    public static boolean hasPose() {
        return currentPose != null;
    }

    /** Call at the end of TeleOp so an old pose doesn't carry into the next match. */
    public static void clear() {
        currentPose = null;
    }
}
