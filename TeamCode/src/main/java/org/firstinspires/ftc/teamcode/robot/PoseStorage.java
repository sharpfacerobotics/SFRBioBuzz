package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;

/**
 * Hands the robot's pose from auto to TeleOp. Owner: Ryan.
 *
 * <p>Static, so it survives from auto into TeleOp as long as the robot app isn't restarted.
 * This is the one subsystem without the RobotHardware constructor - it holds no hardware and
 * has to outlive any single OpMode.</p>
 */
public class PoseStorage {
    private static Pose currentPose = null;
    // Hive state at the end of auto, so TeleOp aims at the right cell (auto usually tips it).
    private static Cell hiveUpCell = null;
    private static double hiveWeight = 0;

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
        hiveUpCell = null;
    }

    /** Call in auto (every loop) with the HiveTracker's state. */
    public static void saveHive(Cell upCell, double weight) {
        hiveUpCell = upCell;
        hiveWeight = weight;
    }

    public static boolean hasHive() {
        return hiveUpCell != null;
    }

    public static Cell getHiveUpCell() {
        return hiveUpCell;
    }

    public static double getHiveWeight() {
        return hiveWeight;
    }
}
