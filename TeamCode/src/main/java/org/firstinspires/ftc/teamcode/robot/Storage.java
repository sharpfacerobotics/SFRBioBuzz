package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.RobotConfig.Element;

import java.util.ArrayDeque;

/**
 * Keeps track of which elements are in the robot and in what order. Owner: Vibhav.
 *
 * <p>A colour/distance sensor at the storage entrance sees each element come in and we
 * classify it by hue. Elements leave first-in-first-out through the launcher; the OpMode
 * calls {@link #removeNext()} when Launcher reports a shot.</p>
 *
 * <p>The launcher needs {@link #peekNext()} to pick the right hood angle/gap for the next
 * element.</p>
 */
public class Storage {
    private final RobotHardware robot;
    private DistanceSensor distanceSensor;

    // ArrayDeque is allocated once; adding/removing enums doesn't allocate.
    private final ArrayDeque<Element> elements = new ArrayDeque<>(RobotConfig.STORAGE_CAPACITY + 1);
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private boolean elementAtEntry = false;
    private Element lastSeen = Element.UNKNOWN;
    private double lastHue = 0;

    public Storage(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Call once per loop. Reads the entry sensor and keeps the count in sync.
     *
     * @param intaking  intake is pulling in - a new element at the entry is counted
     * @param outtaking intake is spitting out - an element leaving the entry is uncounted
     */
    public void update(boolean intaking, boolean outtaking) {
        // The distance read is a slow I2C call (not in the bulk read). Elements can only
        // cross the entry while the intake is moving, so skip it the rest of the time.
        if (!intaking && !outtaking) {
            return;
        }
        if (distanceSensor == null) {
            // The REV colour sensor v3 is also a distance sensor.
            distanceSensor = (DistanceSensor) robot.storageSensor;
        }
        boolean present = distanceSensor.getDistance(DistanceUnit.CM) < RobotConfig.STORAGE_PRESENT_DISTANCE_CM;

        if (outtaking) {
            // Count it out when it clears the sensor on the way out (falling edge).
            if (!present && elementAtEntry) {
                elements.pollLast(); // last in is first out of the intake
            }
        } else if (present && !elementAtEntry && debounceTimer.seconds() > RobotConfig.STORAGE_DEBOUNCE_TIME) {
            // Count on the rising edge only, so one element sitting there counts once.
            debounceTimer.reset();
            lastSeen = classify(robot.storageSensor.getNormalizedColors());
            if (!isFull()) {
                elements.addLast(lastSeen);
            }
        }
        elementAtEntry = present;
    }

    /** Raw entry distance in cm, for SubsystemTest. Slow I2C read - don't call every loop in comp. */
    public double readEntryDistanceCm() {
        if (distanceSensor == null) {
            distanceSensor = (DistanceSensor) robot.storageSensor;
        }
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /** Classify whatever is at the entry right now, for SubsystemTest. */
    public Element readEntryColor() {
        return classify(robot.storageSensor.getNormalizedColors());
    }

    /** Next element to be launched, or null if empty. */
    public Element peekNext() {
        return elements.peekFirst();
    }

    /** Call when an element leaves through the launcher. */
    public void removeNext() {
        elements.pollFirst();
    }

    /** For autos: the robot starts holding preloads the entry sensor never saw. */
    public void preload(int count, Element type) {
        elements.clear();
        for (int i = 0; i < count && i < RobotConfig.STORAGE_CAPACITY; i++) {
            elements.addLast(type);
        }
    }

    /** Driver override if the count ever gets out of sync. */
    public void clear() {
        elements.clear();
    }

    public int count() {
        return elements.size();
    }

    public boolean isFull() {
        return elements.size() >= RobotConfig.STORAGE_CAPACITY;
    }

    public boolean isEmpty() {
        return elements.isEmpty();
    }

    public Element getLastSeen() {
        return lastSeen;
    }

    public double getLastHue() {
        return lastHue;
    }

    private Element classify(NormalizedRGBA color) {
        double hue = hue(color.red, color.green, color.blue);
        if (hue >= 0) {
            lastHue = hue;
        }
        return classifyHue(hue);
    }

    /** Hue in degrees [0, 360), or -1 if the colour is too grey/dark to tell. */
    static double hue(double r, double g, double b) {
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

    static Element classifyHue(double hue) {
        if (hue >= RobotConfig.POLLEN_HUE_MIN && hue <= RobotConfig.POLLEN_HUE_MAX) {
            return Element.POLLEN;
        }
        if (hue >= RobotConfig.NECTAR_HUE_MIN && hue <= RobotConfig.NECTAR_HUE_MAX) {
            return Element.NECTAR;
        }
        return Element.UNKNOWN;
    }
}
