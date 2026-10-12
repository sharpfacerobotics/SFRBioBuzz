package org.firstinspires.ftc.teamcode.robot.helperClasses;

/**
 * Trapezoidal motion profile from a starting velocity to a stop at the goal.
 *
 * <pre>
 *  v |      ______
 *    |     /      \        accelerate (v0 -> peak), cruise, decelerate (peak -> 0)
 * v0 |____/        \___    if there isn't room to reach maxV it becomes a triangle
 *    +----------------- t
 * </pre>
 *
 * Build it ONCE when the goal changes, then call {@link #get(double)} every loop. get() fills
 * and returns the same State object, so nothing is allocated in the loop.
 */
public class MotionProfile {
    public static class State {
        public double position;
        public double velocity;
        public double acceleration;

        private void set(double p, double v, double a) {
            position = p;
            velocity = v;
            acceleration = a;
        }
    }

    private final State state = new State();

    private final double start;
    private final double goal;
    private final double maxA;
    private final double direction;
    private final double v0;
    private final double peakV;
    private final double accelTime;
    private final double cruiseTime;
    private final double decelTime;
    private final double totalTime;
    private final double accelDistance;
    private final double cruiseDistance;

    /**
     * @param start start position
     * @param goal  goal position
     * @param currV current velocity (signed, same units as position per second)
     * @param maxV  max speed (positive)
     * @param maxA  max acceleration (positive)
     */
    public MotionProfile(double start, double goal, double currV, double maxV, double maxA) {
        this.start = start;
        this.goal = goal;
        this.maxA = maxA;

        direction = goal >= start ? 1.0 : -1.0;
        double distance = Math.abs(goal - start);

        // Velocity toward the goal. Moving away (or faster than maxV) is clamped - the
        // profile then assumes the mechanism can reach that instantly.
        v0 = Math.max(0.0, Math.min(maxV, currV * direction));

        // d(v1 -> v2) = (v2^2 - v1^2) / (2a)
        double fullAccelDist = (maxV * maxV - v0 * v0) / (2.0 * maxA);
        double fullDecelDist = (maxV * maxV) / (2.0 * maxA);

        double peak;
        if (fullAccelDist + fullDecelDist <= distance) {
            peak = maxV; // trapezoid
        } else {
            // triangle: (p^2 - v0^2)/2a + p^2/2a = d  ->  p = sqrt(a*d + v0^2/2)
            peak = Math.sqrt(maxA * distance + v0 * v0 / 2.0);
            if (peak < v0) {
                peak = v0; // already too fast to stop in time; decelerate straight away
            }
        }
        peakV = peak;

        accelTime = (peakV - v0) / maxA;
        decelTime = peakV / maxA;
        accelDistance = (v0 + peakV) / 2.0 * accelTime;
        double decelDistance = peakV / 2.0 * decelTime;
        cruiseDistance = Math.max(0.0, distance - accelDistance - decelDistance);
        cruiseTime = peakV > 0 ? cruiseDistance / peakV : 0.0;
        totalTime = accelTime + cruiseTime + decelTime;
    }

    /** Profile state at time t seconds after the profile was built. */
    public State get(double t) {
        if (t <= 0) {
            return transform(0, v0, 0);
        }
        if (t < accelTime) {
            return transform(v0 * t + 0.5 * maxA * t * t, v0 + maxA * t, maxA);
        }
        if (t < accelTime + cruiseTime) {
            double ct = t - accelTime;
            return transform(accelDistance + peakV * ct, peakV, 0);
        }
        if (t < totalTime) {
            double dt = t - accelTime - cruiseTime;
            double x = accelDistance + cruiseDistance + peakV * dt - 0.5 * maxA * dt * dt;
            return transform(x, peakV - maxA * dt, -maxA);
        }
        state.set(goal, 0, 0);
        return state;
    }

    public boolean isFinished(double t) {
        return t >= totalTime;
    }

    public double getTotalTime() {
        return totalTime;
    }

    private State transform(double x, double v, double a) {
        state.set(start + direction * x, direction * v, direction * a);
        return state;
    }
}
