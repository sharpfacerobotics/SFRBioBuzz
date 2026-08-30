package org.firstinspires.ftc.teamcode.robot;


//MUST BE FIXED 
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

    private final double currV;
    private final double direction;
    private final double accelTime;
    private final double cruiseTime;
    private final double totalTime;
    private final double peakV;

    public MotionProfile(
            double start,
            double goal,
            double currV,
            double maxV,
            double maxA) {

        this.start = start;
        this.goal = goal;
        this.currV = currV;
        this.maxV = maxV;
        this.maxA = maxA;

        direction = Math.signum(goal - start);
        currV = currV * direction;
        double distance = Math.abs(goal - start);

        // Distance needed to accelerate then decelerate
        double accelDistance = (maxV * maxV - currV * currV) / maxA;

        if (distance < accelDistance) {
            // Triangle profile: never reaches max velocity
            peakV = Math.sqrt((currV * currV) / 2 + distance * maxA);
            accelTime = (peakV - currV) / maxA;
            cruiseTime = 0;
        } else {
            // Trapezoid profile
            peakV = maxV;
            accelTime = (maxV - currV) / maxA;

            double accelDecelDistance =
                    (maxV * maxV - currV * currV) / maxA;

            cruiseTime =
                    (distance - accelDecelDistance) / peakV;
        }

        totalTime = 2 * accelTime + cruiseTime;
    }

    public State get(double t) {

        if (t < accelTime) {
            // Accelerating
            double a = maxA;
            double v = currV + a * t;
            double x = currV * t + 0.5 * a * t * t;

            return transform(x, v, a);
        }

        if (t < accelTime + cruiseTime) {
            // Constant velocity
            double cruiseT = t - accelTime;

            double accelDist =
                    currV * accelTime + 0.5 * maxA * accelTime * accelTime;

            double x =
                    accelDist + peakV * cruiseT;

            return transform(x, peakV, 0);
        }

        if (t < totalTime) {
            // Decelerating
            double decelT =
                    t - accelTime - cruiseTime;

            double accelDist =
                    currV * accelTime + 0.5 * maxA * accelTime * accelTime;

            double cruiseDist =
                    peakV * cruiseTime;

            double x =
                    accelDist
                            + cruiseDist
                            + peakV * decelT
                            - 0.5 * maxA * decelT * decelT;

            double v =
                    peakV - maxA * decelT;

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
