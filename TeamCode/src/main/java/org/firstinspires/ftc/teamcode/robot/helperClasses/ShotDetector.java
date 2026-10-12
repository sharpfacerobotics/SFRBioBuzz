package org.firstinspires.ftc.teamcode.robot.helperClasses;

/**
 * Detects an element leaving the flywheel from the velocity dip it causes.
 *
 * <p>A shot is a sudden dip: the wheel was at speed, and within {@code window} seconds it fell
 * {@code drop} below the speed it had then. Comparing to the speed we actually had (not the
 * target) means a target change from driving around can't fake a shot, and the time window
 * means a slow drift out of tolerance disarms instead of firing later.</p>
 *
 * <p>Time is passed in so this can be unit tested without a clock.</p>
 */
public class ShotDetector {
    private boolean armed = false;
    private double readyVelocity = 0;
    private double readyTime = 0;

    /**
     * Call once per loop.
     *
     * @param ready    flywheel is at speed right now
     * @param velocity current flywheel velocity
     * @param now      current time (s)
     * @param drop     velocity dip that counts as a shot
     * @param window   seconds after last being ready in which a dip still counts
     * @return true for exactly one loop per shot
     */
    public boolean update(boolean ready, double velocity, double now, double drop, double window) {
        if (ready) {
            armed = true;
            readyVelocity = velocity;
            readyTime = now;
            return false;
        }
        if (!armed) {
            return false;
        }
        if (readyVelocity - velocity > drop) {
            armed = false; // one dip = one shot; re-arms once back at speed
            return true;
        }
        if (now - readyTime > window) {
            armed = false;
        }
        return false;
    }

    public void reset() {
        armed = false;
    }
}
