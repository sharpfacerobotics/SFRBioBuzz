/*
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.AbstractQueue;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Iterator;
import java.util.Objects;
import java.util.Queue;

/**
 * Tracks the single-file storage path. Public method signatures are unchanged.
 * Mouth beams request rejection; only the INNER pair adds/removes stored elements.
 * Both shooter beams are at one station and form one occupied/clear signal.
 
public class Storage {
    public static final String POLLEN = "POLLEN";
    public static final String NECTAR = "NECTAR";

    private final RobotHardware robot;
    private final ElapsedTime clock = new ElapsedTime();
    private final ArrayDeque<String> queue = new ArrayDeque<>(5);
    private final Queue<String> queueView = new ReadOnlyQueue(queue);

    private boolean mouthWas;
    private boolean exitWas;
    private boolean entryActive;
    private boolean entryBeganInward;
    private boolean entrySawHigh;
    private boolean entryColourAttempted;
    private String entryColour;

    private boolean emergency;
    private boolean rejectionDisabledUntilEmpty;
    private boolean rejectionSawMouth;
    private double reversingSeconds;
    private double lastUpdate = Double.NaN;
    private int launchedCount;

    public Storage(RobotHardware robot) {
        this.robot = Objects.requireNonNull(robot, "robot");
    }

    /**
     * UPDATE: once per robot loop, after robot.beginLoop().
     * true requests emergency reverse; false releases that request.
     * The caller owns motor commands, including resuming intake after a timeout.
     
    public boolean checkBeamBreaks(String allianceColor) {
        requireAlliance(allianceColor);
        // With MANUAL caching, these six reads use one bulk snapshot per hub.
        boolean mouthLow = isBroken(robot.intakeMouthBeamLow);
        boolean mouthHigh = isBroken(robot.intakeMouthBeamHigh);
        boolean innerLow = isBroken(robot.intakeQueueBeamLow);
        boolean innerHigh = isBroken(robot.intakeQueueBeamHigh);
        boolean exitLow = isBroken(robot.storageExitBeamLow);
        boolean exitHigh = isBroken(robot.storageExitBeamHigh);

        boolean inward = robot.intakePowerCommand > 0;
        boolean inboundPassage = entryActive ? entryBeganInward : innerLow || innerHigh;
        boolean nectarSeen = innerHigh || (entryActive && entrySawHigh);
        boolean colourAttempted = inward && inboundPassage && nectarSeen
                && !entryColourAttempted;
        String colour = null;
        if (colourAttempted) {
            // Initial read + TWO retries. No sleep, no motor stop, stop on a clear answer.
            // RGB is obtained together; repeated reads are not guaranteed fresh conversions.
            for (int attempt = 0; attempt < 3; attempt++) {
                colour = readNectarColor();
                if (colour != null) break;
            }
        }

        return process(mouthLow, mouthHigh, innerLow, innerHigh, exitLow, exitHigh,
                robot.intakePowerCommand, robot.shooterFeederPowerCommand,
                colourAttempted, colour, allianceColor, clock.seconds());
    }

    /** Hardware-independent transition function; package access for regression tests. 
    boolean process(boolean mouthLow, boolean mouthHigh, boolean innerLow, boolean innerHigh,
                    boolean exitLow, boolean exitHigh, double intakePower, double feederPower,
                    boolean colourAttempted, String nectarColour, String allianceColor,
                    double now) {
        requireAlliance(allianceColor);
        boolean mouth = mouthLow || mouthHigh;
        boolean inner = innerLow || innerHigh;
        boolean exit = exitLow || exitHigh;
        boolean inward = intakePower > 0;
        boolean outward = intakePower < 0;

        // Measure actual commanded reverse time, not time since detection.
        // The caller applies commands AFTER checkBeamBreaks(): the command sampled
        // now is the command that moved elements during the preceding loop interval.
        if (emergency && outward && Double.isFinite(lastUpdate)) {
            reversingSeconds += Math.max(0, now - lastUpdate);
        }

        // Shooter direction is independent of intake direction. A high-beam transition
        // alone is not an exit while the low beam is still blocked (and vice versa).
        if (exitWas && !exit && feederPower > 0 && queue.pollFirst() != null) {
            launchedCount++;
        }

        // After a timeout, do not immediately retrigger on the same blocked element.
        // Rearm when storage is empty and no intake passage is still in progress.
        if (rejectionDisabledUntilEmpty && queue.isEmpty() && !entryActive && !mouth && !inner) {
            rejectionDisabledUntilEmpty = false;
        }

        boolean startedInbound = false;
        if (!entryActive && inner && intakePower != 0) {
            entryActive = true;
            entryBeganInward = inward;
            entrySawHigh = false;
            entryColourAttempted = false;
            entryColour = null;
            startedInbound = inward;
        }

        if (entryActive) {
            // Remember every high-beam hit until the entire passage finishes.
            entrySawHigh |= innerHigh;
            if (entryBeganInward && colourAttempted) {
                entryColourAttempted = true;
                entryColour = nectarColour; // null after three unclear reads is allowed NECTAR.
            }
        }

        if (inward && !rejectionDisabledUntilEmpty && RobotConfig.emergencyConfigured()) {
            boolean fullArrival = queue.size() >= RobotConfig.STORAGE_CAPACITY
                    && ((mouth && !mouthWas) || startedInbound);
            boolean opponent = entryActive && entryBeganInward && entrySawHigh
                    && entryColour != null && !entryColour.equals(allianceColor);
            if (!emergency && (fullArrival || opponent)) startEmergency(mouth);
        }

        if (entryActive && !inner && intakePower != 0) {
            if (entryBeganInward && inward) {
                // Count what physically entered, INCLUDING a fifth element. Never
                // publish provisional POLLEN and then replace it with NECTAR.
                queue.addLast(entrySawHigh ? NECTAR : POLLEN);
            } else if (!entryBeganInward && outward) {
                queue.pollLast(); // An already-stored element crossed back out.
            }
            // Inbound then reversed out: cancel, without adding or removing a stored item.
            // Outbound then returned inward: the stored item was never removed.
            entryActive = false;
            entrySawHigh = false;
            entryColourAttempted = false;
            entryColour = null;
        }

        if (emergency) {
            // A ball rejected at the inner station must return through the mouth.
            // Merely clearing the inner pair could mean it moved farther inward.
            if (outward && mouth) rejectionSawMouth = true;
            boolean returned = outward && rejectionSawMouth && !mouth && !inner
                    && reversingSeconds >= RobotConfig.EMERGENCY_MIN_OUTTAKE;
            if (returned) {
                emergency = false; // A successful rejection does not disable later rejections.
            } else if (reversingSeconds >= 3.0 * RobotConfig.EMERGENCY_MAX_OUTTAKE) {
                emergency = false;
                rejectionDisabledUntilEmpty = true;
            }
        }

        mouthWas = mouth;
        exitWas = exit;
        lastUpdate = now;
        return emergency;
    }

    private void startEmergency(boolean mouth) {
        emergency = true;
        reversingSeconds = 0;
        rejectionSawMouth = mouth;
    }

    /** STATUS: confirmed forward departures from storage into the shooter, not scored shots. 
    public int getLaunchedCount() {
        return launchedCount;
    }

    /** STATUS: either mouth beam OR either inner intake beam is broken. 
    public boolean anyIntakeBeamBroken() {
        return isBroken(robot.intakeMouthBeamLow) || isBroken(robot.intakeMouthBeamHigh)
                || isBroken(robot.intakeQueueBeamLow) || isBroken(robot.intakeQueueBeamHigh);
    }

    /** STATUS: live, read-only FIFO view. peek()/size() allocate nothing. 
    public Queue<String> getElementQueue() {
        return queueView;
    }

    /** INIT: replaces preload contents and resets passage/rejection/exit-count state. 
    public void preloadQueue(int count, String element) {
        if (count < 0 || count > RobotConfig.STORAGE_CAPACITY) {
            throw new IllegalArgumentException("Preload count must be 0.." + RobotConfig.STORAGE_CAPACITY);
        }
        if (!POLLEN.equals(element) && !NECTAR.equals(element)) {
            throw new IllegalArgumentException("Element must be POLLEN or NECTAR");
        }
        queue.clear();
        for (int i = 0; i < count; i++) queue.addLast(element);
        mouthWas = exitWas = entryActive = entryBeganInward = entrySawHigh = false;
        entryColourAttempted = emergency = rejectionDisabledUntilEmpty = false;
        rejectionSawMouth = false;
        entryColour = null;
        reversingSeconds = 0;
        lastUpdate = Double.NaN;
        launchedCount = 0;
        clock.reset();
    }

    /** One I2C color read. RED / BLUE / null; uncalibrated settings return null. 
    public String readNectarColor() {
        if (!RobotConfig.colourConfigured()) return null;
        NormalizedRGBA c = robot.intakeColorSensor.getNormalizedColors();
        double sum = c.red + c.green + c.blue;
        if (!Double.isFinite(sum) || sum <= RobotConfig.STORAGE_MIN_RGB_SUM) return null;
        return nectarColorFromHue(hue(c.red, c.green, c.blue));
    }

    private static boolean isBroken(DigitalChannel beam) {
        return !beam.getState();
    }

    /** Hue [0, 360), or -1 for invalid/grey/insufficiently saturated input. 
    public static double hue(double r, double g, double b) {
        if (!Double.isFinite(r) || !Double.isFinite(g) || !Double.isFinite(b)
                || r < 0 || g < 0 || b < 0) return -1;
        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        double delta = max - min;
        double minimumSaturation = Double.isFinite(RobotConfig.STORAGE_MIN_SATURATION)
                ? RobotConfig.STORAGE_MIN_SATURATION : 0;
        if (max <= 0 || delta <= 0 || delta / max < minimumSaturation) return -1;
        double result;
        if (max == r) result = 60.0 * ((g - b) / delta);
        else if (max == g) result = 60.0 * ((b - r) / delta + 2.0);
        else result = 60.0 * ((r - g) / delta + 4.0);
        return result < 0 ? result + 360 : result;
    }

    static String nectarColorFromHue(double hue) {
        if (!Double.isFinite(hue) || hue < 0 || hue >= 360) return null;
        boolean blue = inHueWindow(hue, RobotConfig.BLUE_NECTAR_HUE_MIN,
                RobotConfig.BLUE_NECTAR_HUE_MAX);
        boolean red = inHueWindow(hue, RobotConfig.RED_NECTAR_HUE_MIN,
                RobotConfig.RED_NECTAR_HUE_MAX);
        if (blue == red) return null; // Neither window, or ambiguous overlapping windows.
        return blue ? "BLUE" : "RED";
    }

    private static boolean inHueWindow(double hue, double min, double max) {
        if (!Double.isFinite(min) || !Double.isFinite(max)) return false;
        return min <= max ? hue >= min && hue <= max : hue >= min || hue <= max;
    }

    private static void requireAlliance(String alliance) {
        if (!"RED".equals(alliance) && !"BLUE".equals(alliance)) {
            throw new IllegalArgumentException("allianceColor must be RED or BLUE");
        }
    }

    private static final class ReadOnlyQueue extends AbstractQueue<String> {
        private final ArrayDeque<String> source;
        ReadOnlyQueue(ArrayDeque<String> source) { this.source = source; }
        @Override public String peek() { return source.peekFirst(); }
        @Override public int size() { return source.size(); }
        @Override public Iterator<String> iterator() {
            return Collections.unmodifiableCollection(source).iterator();
        }
        @Override public boolean offer(String value) {
            throw new UnsupportedOperationException("Storage owns the queue");
        }
        @Override public String poll() {
            throw new UnsupportedOperationException("Storage owns the queue");
        }
        @Override public void clear() {
            throw new UnsupportedOperationException("Storage owns the queue");
        }
    }
}

*/