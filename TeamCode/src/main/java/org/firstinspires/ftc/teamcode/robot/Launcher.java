package org.firstinspires.ftc.teamcode.robot;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Queue;

@Configurable
public class Launcher {
    public static double kF = 0.00045;
    public static double kP = 0.0123; // these contants gotta move to RobotConfig, but they're not done being tuned yet
    public static boolean usePF = true;
    public static double nominalVoltage = 12.5;
    private double currVoltage;
    private boolean preShooterBeamBreakBroken = false;
    private boolean preShooterBeamBreakWasBroken = false;
    private ElapsedTime preShooterBeamBreakLagTimer = new ElapsedTime();
    private final ElapsedTime hoodMovementTimer = new ElapsedTime();
    private RobotHardware robot;
    public static double BASE_FLYWHEEL_RVEL = 1200;
    private double targetVel;
    private double currVel1;
    public static double hoodRetractionPos;
    public static double hoodAnglePos;


    private enum FlywheelState {
        IDLE,
        PF_CONTROLLED,
        NO_TOLERANCE,
        LINGERING_STRICT
    }
    private FlywheelState flywheelState = FlywheelState.IDLE;

    public Launcher(RobotHardware robot) {
        this.robot = robot;
        //robot.launchMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //comp method

    public void setLauncher(Queue<String> q) {
        currVoltage = robot.voltageSensor.getVoltage();
        String next = q.peek();
        double vx = ShooterConfig.elementVelocity[0];
        double vy = ShooterConfig.elementVelocity[1];

        targetVel = Math.hypot(vx, vy);

        currVel1 = robot.launchMotor1.getVelocity();
        if (next != null) {
            setHoodRetraction(next);
            setHoodAngle(next);
        }
        switch (flywheelState) {
            case IDLE:
                // if intake beam breaks broken
                // change to pf controlled
                if (!q.isEmpty()) {
                    flywheelState = FlywheelState.PF_CONTROLLED;
                }

                break;
            case PF_CONTROLLED:
                /*
                if (preShooterBeamBreakBroken) {
                    flywheelState = FlywheelState.NO_TOLERANCE;
                }
                */
                if (q.isEmpty()) {
                    flywheelState = FlywheelState.IDLE;
                }
                break;
            case NO_TOLERANCE:
                if (!preShooterBeamBreakBroken && preShooterBeamBreakWasBroken) {
                    flywheelState = FlywheelState.LINGERING_STRICT;
                }
                preShooterBeamBreakLagTimer.reset();
                break;
            case LINGERING_STRICT:
                if (preShooterBeamBreakLagTimer.seconds() > 0.2) {
                    preShooterBeamBreakLagTimer.reset();
                    if (q.isEmpty()) {
                        flywheelState = FlywheelState.IDLE;
                    } else {
                        flywheelState = FlywheelState.PF_CONTROLLED;
                    }
                }
                break;
            default:
                flywheelState = FlywheelState.PF_CONTROLLED;
        }

        //preShooterBeamBreakWasBroken = preShooterBeamBreakBroken;

        this.targetVel = targetVel;
        setVelocityPF();
    }


    //testing method
    public void setLauncher(double targetVel, Queue<String> q) {
        currVoltage = robot.voltageSensor.getVoltage();
        //preShooterBeamBreakBroken = !robot.preShooterBeamBreak.getState();

        switch (flywheelState) {
            case IDLE:
                // if intake beam breaks broken
                // change to pf controlled
                if (!q.isEmpty()) {
                    flywheelState = FlywheelState.PF_CONTROLLED;
                }

                break;
            case PF_CONTROLLED:
                /*
                if (preShooterBeamBreakBroken) {
                    flywheelState = FlywheelState.NO_TOLERANCE;
                }
                */
                if (q.isEmpty()) {
                    flywheelState = FlywheelState.IDLE;
                }
                break;
            case NO_TOLERANCE:
                if (!preShooterBeamBreakBroken && preShooterBeamBreakWasBroken) {
                    flywheelState = FlywheelState.LINGERING_STRICT;
                }
                preShooterBeamBreakLagTimer.reset();
                break;
            case LINGERING_STRICT:
                if (preShooterBeamBreakLagTimer.seconds() > 0.2) {
                    preShooterBeamBreakLagTimer.reset();
                    if (q.isEmpty()) {
                        flywheelState = FlywheelState.IDLE;
                    } else {
                        flywheelState = FlywheelState.PF_CONTROLLED;
                    }
                }
                break;
            default:
                flywheelState = FlywheelState.PF_CONTROLLED;
        }

        //preShooterBeamBreakWasBroken = preShooterBeamBreakBroken;

        currVel1 = robot.launchMotor1.getVelocity();
        this.targetVel = targetVel;
        //String next = q.peek();
        //setHoodRetraction(next);
        //setHoodAngle(next);
        setVelocityPF();
    }
    //testing method, no storage
    public void setLauncher(double targetVel) {
        currVoltage = robot.voltageSensor.getVoltage();
        flywheelState = FlywheelState.PF_CONTROLLED;
        currVel1 = robot.launchMotor1.getVelocity();
        this.targetVel = targetVel;
        //String next = q.peek();
        //setHoodRetraction(next);
        //setHoodAngle(next);
        setVelocityPF();
    }

    private void setVelocityPF() {
        double pow1;
        if (flywheelState == FlywheelState.IDLE) {
            pow1 = kF * BASE_FLYWHEEL_RVEL + kP * (BASE_FLYWHEEL_RVEL - currVel1);
        } else if (flywheelState == FlywheelState.PF_CONTROLLED){
            pow1 = kF * targetVel + kP * (targetVel - currVel1);
        } else if (flywheelState == FlywheelState.NO_TOLERANCE || flywheelState == FlywheelState.LINGERING_STRICT) {
            pow1 = kF * targetVel + 0.5 * Math.max((targetVel - currVel1), 0);
        } else {
            pow1 = kF * targetVel + kP * (targetVel - currVel1);
        }
        if (Double.isFinite(currVoltage) && currVoltage > 8.0) {
            pow1 *= nominalVoltage / currVoltage;
        }
        robot.launchMotor1.setPower(Range.clip(pow1, 0, 1));
    }
    public void setkP(double kP) {
        Launcher.kP = kP;
    }
    public void setkF(double kF) {
        Launcher.kF = kF;
    }

    public double getCurrVelocity() {
        return currVel1;
    }
    //testing method

    public void setHoodAngle() {
        if (robot.hoodAngleServo.getPosition() != hoodAnglePos) {
            hoodMovementTimer.reset();
        }
        robot.hoodAngleServo.setPosition(hoodAnglePos);
    }
    public void setHoodRetraction() {
        if (robot.hoodGapServo1.getPosition() != hoodRetractionPos
                || robot.hoodGapServo2.getPosition() != hoodRetractionPos) {
            hoodMovementTimer.reset();
        }
        robot.hoodGapServo1.setPosition(hoodRetractionPos);
        robot.hoodGapServo2.setPosition(hoodRetractionPos);
    }
    public double getHoodAnglePos() {
        return hoodAnglePos;
    }
    public double getHoodRetractionPos() {
        return hoodRetractionPos;
    }

    public void setHoodAngle(String next) {
        if (next.equals("POLLEN")) {
            hoodAnglePos = RobotConfig.HOOD_POLLEN_ANGLE_POS;
        } else if (next.equals("NECTAR")) {
            hoodAnglePos = RobotConfig.HOOD_NECTAR_ANGLE_POS;
        }
        setHoodAngle();
    }

    public void setHoodRetraction(String next) {
        if (next.equals("POLLEN")) {
            hoodRetractionPos = RobotConfig.HOOD_POLLEN_GAP;
        } else if (next.equals("NECTAR")) {
            hoodRetractionPos = RobotConfig.HOOD_NECTAR_GAP;
        }
        setHoodRetraction();
    }

    public boolean goodShooterConfig() {
        boolean hoodRetracted = Math.abs((robot.hoodGapServo1.getPosition() + robot.hoodGapServo2.getPosition()) / 2 - hoodRetractionPos) < RobotConfig.HOOD_RETRACTION_TOLERANCE;
        boolean flywheelSpunUp = Math.abs(targetVel - currVel1) < RobotConfig.FLYWHEEL_VELOCITY_TOLERANCE;
        boolean hoodAngleSet = Math.abs(robot.hoodAngleServo.getPosition() - hoodAnglePos) < RobotConfig.HOOD_ANGLE_TOLERANCE;
        return hoodRetracted && flywheelSpunUp && hoodAngleSet && hoodMovementTimer.seconds() >= RobotConfig.HOOD_MOVEMENT_TIME;
    }
}
