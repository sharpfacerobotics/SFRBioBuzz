package org.firstinspires.ftc.teamcode.robot.helperClasses;

public class MotionProfile {

    public static class State {
        public double position;
        public double velocity;
        public double acceleration;

        public State(double p, double v, double a) {
            position = p;
            velocity = v;
            acceleration = a;
        }
    }

    private final double start;
    private final double goal;
    private final double maxV;
    private final double maxA;
    // Starting speed along the direction of travel, clamped to [0, maxV]
    private final double currV;
    private final double direction;

    private final double accelTime;
    private final double cruiseTime;
    private final double decelTime;
    private final double totalTime;
    private final double peakV;
    private final double accelDist;
    private final double cruiseDist;

    public MotionProfile(
            double start,
            double goal,
            double currV,
            double maxV,
            double maxA) {
        if (maxV <= 0 || maxA <= 0) {
            throw new IllegalArgumentException("maxV and maxA must be positive.");
        }
        this.start = start;
        this.goal = goal;
        this.maxV = maxV;
        this.maxA = maxA;

        direction = Math.signum(goal - start);
        this.currV = Math.max(0, Math.min(maxV, currV * direction));

        double distance = Math.abs(goal - start);
        double v0 = this.currV;

        // Distance to speed up from v0 to maxV, then slow from maxV to 0
        double fullProfileDistance = (2 * maxV * maxV - v0 * v0) / (2 * maxA);

        if (distance < fullProfileDistance) {
            // Triangle profile: never reaches max velocity
            peakV = Math.max(v0, Math.sqrt(maxA * distance + v0 * v0 / 2));
        } else {
            // Trapezoid profile
            peakV = maxV;
        }
        accelTime = (peakV - v0) / maxA;
        decelTime = peakV / maxA;
        accelDist = (peakV * peakV - v0 * v0) / (2 * maxA);
        double decelDist = peakV * peakV / (2 * maxA);
        cruiseDist = Math.max(0, distance - accelDist - decelDist);
        cruiseTime = peakV > 0 ? cruiseDist / peakV : 0;
        totalTime = accelTime + cruiseTime + decelTime;
    }

    public State get(double t) {
        if (t < accelTime) {
            // Accelerating
            double v = currV + maxA * t;
            double x = currV * t + 0.5 * maxA * t * t;
            return transform(x, v, maxA);
        }

        if (t < accelTime + cruiseTime) {
            // Constant velocity
            double x = accelDist + peakV * (t - accelTime);
            return transform(x, peakV, 0);
        }

        if (t < totalTime) {
            // Decelerating
            double decelT = t - accelTime - cruiseTime;
            double x = accelDist + cruiseDist
                    + peakV * decelT
                    - 0.5 * maxA * decelT * decelT;
            double v = peakV - maxA * decelT;
            return transform(x, v, -maxA);
        }

        return new State(goal, 0, 0);
    }

    private State transform(double x, double v, double a) {
        return new State(
                start + direction * x,
                direction * v,
                direction * a
        );
    }

    public boolean isFinished(double t) {
        return t >= totalTime;
    }
}
