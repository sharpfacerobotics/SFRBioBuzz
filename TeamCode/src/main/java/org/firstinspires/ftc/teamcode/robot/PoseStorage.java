package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;

import java.util.Queue;

/**
 * Hands the robot's state from auto to TeleOp. Owner: Ryan.
 *
 * <p>Glossary: {@link #updatePoseStoragePose(Pose)} (SET) and {@link #getPoseStoragePose()}
 * (STATUS). Beyond the pose it also carries what TeleOp would otherwise get wrong at the start:
 * which hive cell is up, what's in storage (G407 count), and where the turret's zero is.</p>
 *
 * <p>Static, so it survives from auto into TeleOp as long as the robot app isn't restarted.
 * Everything is stamped with the alliance and time, and {@link #isFresh(String)} rejects state
 * from the other alliance or from a run long ago (e.g. a pit auto before a match with no auto).</p>
 */
public class PoseStorage {
    /** Auto end -> TeleOp init is ~8 s transition + init; anything older than this is stale. */
    public static double MAX_AGE_SECONDS = 120;

    private static Pose currentPose = null;
    private static String alliance = null;
    private static long savedAtNanos = 0;

    private static Cell hiveUpCell = null;
    private static double hiveWeight = 0;
    private static boolean hasTurretZero = false;
    private static double turretZeroTicks = 0;
    // Fixed-size copy of the storage queue - no allocation when saving every loop.
    private static final String[] elements = new String[RobotConfig.STORAGE_CAPACITY];
    private static int elementCount = 0;

    private PoseStorage() {
    }

    /**
     * SET. Call in auto with follower.pose(). Best: call it EVERY loop in auto, so the pose is
     * saved even if the 30s timer cuts auto off. (Pose is immutable, so storing it is safe.)
     */
    public static void updatePoseStoragePose(Pose currPose) {
        currentPose = currPose;
        savedAtNanos = System.nanoTime();
    }

    /** STATUS. The saved auto pose for TeleOp. Null if auto never ran. */
    public static Pose getPoseStoragePose() {
        return currentPose;
    }

    /** Call every loop in auto alongside the pose (no allocation). */
    public static void updateMatchState(String allianceColor, Cell upCell, double weight, double turretZero) {
        alliance = allianceColor;
        hiveUpCell = upCell;
        hiveWeight = weight;
        hasTurretZero = true;
        turretZeroTicks = turretZero;
    }

    /** Call once when auto ends: what's still in storage (TeleOp's G407 count starts from it). */
    public static void updateStoredElements(Queue<String> storage) {
        elementCount = 0;
        for (String e : storage) {
            if (elementCount < elements.length) {
                elements[elementCount++] = e;
            }
        }
    }

    /** Check this before using anything here in TeleOp. */
    public static boolean isFresh(String allianceColor) {
        return currentPose != null
                && allianceColor.equals(alliance)
                && (System.nanoTime() - savedAtNanos) / 1e9 < MAX_AGE_SECONDS;
    }

    public static boolean hasPose() {
        return currentPose != null;
    }

    public static Cell getHiveUpCell() {
        return hiveUpCell;
    }

    public static double getHiveWeight() {
        return hiveWeight;
    }

    public static boolean hasTurretZero() {
        return hasTurretZero;
    }

    public static double getTurretZeroTicks() {
        return turretZeroTicks;
    }

    public static int getElementCount() {
        return elementCount;
    }

    public static String getElement(int i) {
        return elements[i];
    }

    /** Call at the end of TeleOp so an old state doesn't carry into the next match. */
    public static void clear() {
        currentPose = null;
        alliance = null;
        hiveUpCell = null;
        hiveWeight = 0;
        hasTurretZero = false;
        elementCount = 0;
    }
}
