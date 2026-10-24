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
    private final ElapsedTime debounceTimer = new ElapsedTime();

    private boolean intakeLowWas = false;
    private boolean exitWas = false;
    /** An element we must not keep is in the intake; stays true until the beams clear. */
    private boolean emergency = false;

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
        boolean outtaking = robot.intakeMotor.getPower() < 0;

        // The colour sensor is a slow I2C read, so only read it when a NECTAR is coming in.
        String nectarColor = null;
        if (low && high && !intakeLowWas && !outtaking) {
            nectarColor = readNectarColor();
        }
        return process(low, high, exit, outtaking, nectarColor, allianceColor, debounceTimer.seconds());
    }

    /**
     * The queue logic, kept separate from the hardware reads so it can be unit tested.
     *
     * @param nectarColor "RED" / "BLUE" / null (not read this loop or unclear)
     * @param now         seconds, for debouncing
     */
    boolean process(boolean low, boolean high, boolean exit, boolean outtaking,
                    String nectarColor, String allianceColor, double now) {
        if (!outtaking && low && !intakeLowWas && now > RobotConfig.STORAGE_DEBOUNCE_TIME) {
            debounceTimer.reset();
            // Rising edge: an element is entering.
            boolean isNectar = high;
            if (isNectar && nectarColor != null && !nectarColor.equals(allianceColor)) {
                emergency = true; // G408: opponent NECTAR
            } else if (queue.size() >= RobotConfig.STORAGE_CAPACITY) {
                emergency = true; // G407: a 5th element
            } else {
                queue.add(isNectar ? NECTAR : POLLEN);
            }
        } else if (outtaking && !low && intakeLowWas && !emergency) {
            // Falling edge while spitting out: a counted element left through the intake.
            queue.pollLast(); // last in is first out of the intake
        }

        // The rejected element is gone once both intake beams are clear.
        if (emergency && !low && !high) {
            emergency = false;
        }

        // An element leaving the exit beam has gone into the launcher.
        if (exitWas && !exit) {
            queue.poll();
        }

        intakeLowWas = low;
        exitWas = exit;
        return emergency;
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
