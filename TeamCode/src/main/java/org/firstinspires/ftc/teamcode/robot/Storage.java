package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.Queue;

/**
 * Keeps track of which elements are in the robot and in what order. Owner: Vibhav.
 *
 * <p>Glossary methods (Software Documentation, "Storage"):</p>
 * <ul>
 *   <li>{@link #checkBeamBreaks(String)}: UPDATE, returns boolean. Loop call. Updates the
 *       queue; true = run {@code Intake.emergencyOuttake()}.</li>
 *   <li>{@link #anyIntakeBeamBroken()}: STATUS, boolean.</li>
 *   <li>{@link #getElementQueue()}: STATUS, {@code Queue<String>}. Elements in intake order,
 *       "POLLEN" or "NECTAR". The Launcher uses {@code .peek()} for the next element.</li>
 * </ul>
 *
 * <p>Sensors (names in RobotConfig):</p>
 * <ul>
 *   <li>Two beam breaks at the intake mouth. LOW breaks for any element; HIGH sits above a
 *       POLLEN (2.8 in) but below a NECTAR (3.6 in), so it only breaks for NECTAR.</li>
 *   <li>One beam break at the storage exit. An element leaving into the launcher clears it.</li>
 *   <li>The intake colour sensor, read only when a NECTAR comes in, to tell red from blue.</li>
 * </ul>
 *
 * <p>BIOBUZZ rules: never keep the opponent's NECTAR (G408), and never take a 5th element
 * (G407). Both make checkBeamBreaks return true until the element is back out.</p>
 */
public class Storage {
    public static final String POLLEN = "POLLEN";
    public static final String NECTAR = "NECTAR";

    private final RobotHardware robot;

    // Allocated once; adding/removing String constants doesn't allocate.
    private final ArrayDeque<String> queue = new ArrayDeque<>(RobotConfig.STORAGE_CAPACITY + 1);
    private final ElapsedTime clock = new ElapsedTime();

    private boolean intakeLowWas = false;
    private boolean exitWas = false;
    private double lastEntryTime = -1e9;

    /** An element we must not keep is in the intake (G407 / G408). */
    private boolean emergency = false;
    private double emergencyStart = 0;

    /** A NECTAR is at the intake but the colour sensor hasn't given a clear answer yet. */
    private boolean colourPending = false;
    /** The element that just entered was queued as POLLEN and is still on the low beam. */
    private boolean entryQueuedAsPollen = false;

    private int launchedCount = 0;

    public Storage(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * UPDATE. Call once per loop.
     *
     * @param allianceColor "RED" or "BLUE". Opponent-coloured NECTAR is rejected (G408).
     * @return true = emergency outtake needed (opponent NECTAR or a 5th element is in the intake)
     */
    public boolean checkBeamBreaks(String allianceColor) {
        boolean low = isBroken(robot.intakeBeamLow);
        boolean high = isBroken(robot.intakeBeamHigh);
        boolean exit = isBroken(robot.storageExitBeam);
        // Negative intake power = rollers spitting out, so intake beam edges mean "leaving".
        // (The last commanded power - reading intakeMotor.getPower() would cost a hub round trip.)
        boolean outtaking = robot.intakePowerCommand < 0;

        // The colour sensor is a slow I2C read, so only read it while a NECTAR is at the intake:
        // on its way in, or while we're still waiting for a clear colour.
        String nectarColor = null;
        if (high && !outtaking && (colourPending || !intakeLowWas || entryQueuedAsPollen)) {
            nectarColor = readNectarColor();
        }
        return process(low, high, exit, outtaking, nectarColor, allianceColor, clock.seconds());
    }

    /**
     * The queue logic, kept separate from the hardware reads so it can be unit tested.
     *
     * @param nectarColor "RED" / "BLUE" / null (not read this loop, or unclear)
     * @param now         seconds (any monotonic clock)
     */
    boolean process(boolean low, boolean high, boolean exit, boolean outtaking,
                    String nectarColor, String allianceColor, double now) {
        boolean rising = !outtaking && low && !intakeLowWas
                && now - lastEntryTime > RobotConfig.STORAGE_DEBOUNCE_TIME;

        if (rising) {
            lastEntryTime = now;
            if (queue.size() >= RobotConfig.STORAGE_CAPACITY) {
                startEmergency(now); // G407: a 5th element
            } else if (high) {
                colourPending = true; // NECTAR: decide below once the colour is known
            } else {
                queue.add(POLLEN);
                entryQueuedAsPollen = true;
            }
        } else if (!outtaking && entryQueuedAsPollen && high) {
            // The high beam broke a loop or two after the low one: it's actually a NECTAR.
            queue.pollLast();
            entryQueuedAsPollen = false;
            colourPending = true;
        } else if (outtaking && !low && intakeLowWas && !emergency && !colourPending) {
            // Falling edge while spitting out: a counted element left through the intake.
            queue.pollLast(); // last in is first out of the intake
        }

        if (colourPending) {
            if (nectarColor != null) {
                colourPending = false;
                if (nectarColor.equals(allianceColor)) {
                    queue.add(NECTAR);
                } else {
                    startEmergency(now); // G408: opponent NECTAR
                }
            } else if (!low && !high) {
                // Passed the sensor without a clear colour. Keep it rather than spit out what
                // is most likely our own NECTAR - tune the hue windows if this shows up.
                colourPending = false;
                queue.add(NECTAR);
            }
        }
        if (!low) {
            entryQueuedAsPollen = false;
        }

        // The rejected element is gone once it has been pushed back out past both beams
        // (seen while outtaking, after a minimum outtake). Give up after a maximum so a ball
        // that went inward can't leave the intake stuck reversing.
        if (emergency) {
            double t = now - emergencyStart;
            boolean pushedOut = outtaking && !low && !high && t > RobotConfig.EMERGENCY_MIN_OUTTAKE;
            if (pushedOut || t > RobotConfig.EMERGENCY_MAX_OUTTAKE) {
                emergency = false;
            }
        }

        // An element leaving the exit beam (while not reversing) has gone into the launcher.
        if (!outtaking && exitWas && !exit) {
            queue.poll();
            launchedCount++;
        }

        intakeLowWas = low;
        exitWas = exit;
        return emergency;
    }

    private void startEmergency(double now) {
        emergency = true;
        emergencyStart = now;
    }

    /**
     * STATUS. Total elements that have left storage into the launcher this OpMode. The OpMode
     * compares it loop to loop to count shots (jitter/outtake never change it).
     */
    public int getLaunchedCount() {
        return launchedCount;
    }

    /** STATUS. True if any INTAKE beam is broken. */
    public boolean anyIntakeBeamBroken() {
        return isBroken(robot.intakeBeamLow) || isBroken(robot.intakeBeamHigh);
    }

    /**
     * STATUS. Stored elements in intake order ("POLLEN" / "NECTAR"). This is the live queue:
     * {@code .peek()} is the next element to launch, {@code .size()} the count.
     */
    public Queue<String> getElementQueue() {
        return queue;
    }

    /**
     * INIT (auto only). The robot starts holding pre-loads that never crossed the intake beams
     * (G304: 4 POLLEN).
     */
    public void preloadQueue(int count, String element) {
        queue.clear();
        for (int i = 0; i < count && i < RobotConfig.STORAGE_CAPACITY; i++) {
            queue.add(element);
        }
    }

    /** "RED", "BLUE", or null if the colour sensor can't tell. Slow I2C read. */
    public String readNectarColor() {
        NormalizedRGBA c = robot.intakeColorSensor.getNormalizedColors();
        return nectarColorFromHue(hue(c.red, c.green, c.blue));
    }

    /** Beam break sensors read LOW (false) when the beam is broken. */
    private static boolean isBroken(DigitalChannel beam) {
        return !beam.getState();
    }

    /** Hue in degrees [0, 360), or -1 if the colour is too grey/dark to tell. */
    public static double hue(double r, double g, double b) {
        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        double delta = max - min;
        if (max <= 0 || delta / max < RobotConfig.STORAGE_MIN_SATURATION) {
            return -1;
        }
        double hue;
        if (max == r) {
            hue = 60.0 * ((g - b) / delta);
        } else if (max == g) {
            hue = 60.0 * ((b - r) / delta + 2.0);
        } else {
            hue = 60.0 * ((r - g) / delta + 4.0);
        }
        return hue < 0 ? hue + 360.0 : hue;
    }

    /** "RED" / "BLUE" NECTAR from hue, or null. */
    static String nectarColorFromHue(double hue) {
        if (hue < 0) {
            return null;
        }
        if (hue >= RobotConfig.BLUE_NECTAR_HUE_MIN && hue <= RobotConfig.BLUE_NECTAR_HUE_MAX) {
            return "BLUE";
        }
        // red wraps around 0 degrees
        if (hue >= RobotConfig.RED_NECTAR_HUE_MIN || hue <= RobotConfig.RED_NECTAR_HUE_MAX) {
            return "RED";
        }
        return null;
    }
}
