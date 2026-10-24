package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Turret;

/**
 * Turret PD tuning. Homes on the hall effect, then either holds TARGET_DEG or (STEP_MODE)
 * jumps between TARGET_DEG and STEP_TARGET_DEG every STEP_PERIOD seconds so you can watch the
 * step response in the Panels graph. Edit RobotConfig.TURRET_KP / KD / KS live in Panels.
 *
 * <p>Tuning order: KS until it just starts moving, KP until it overshoots a little, then KD
 * until the overshoot is gone.</p>
 */
@Configurable
@TeleOp(name = "TurretTuner", group = "Test")
public class TurretTuner extends OpMode {
    public static double TARGET_DEG = 0;
    public static boolean STEP_MODE = false;
    public static double STEP_TARGET_DEG = 90;
    public static double STEP_PERIOD = 2.0;

    private final RobotHardware robot = new RobotHardware();
    private final Turret turret = new Turret(robot);
    private final ElapsedTime stepTimer = new ElapsedTime();
    private boolean onStep = false;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        robot.init(hardwareMap);
        telemetry.addLine("Press start - the turret will home first.");
    }

    @Override
    public void start() {
        turret.localizeTurret();
        turret.restartTimer();
        stepTimer.reset();
    }

    @Override
    public void loop() {
        robot.clearBulkCache();
        if (gamepad1.aWasPressed()) {
            turret.localizeTurret();
        }
        if (STEP_MODE && stepTimer.seconds() > STEP_PERIOD) {
            stepTimer.reset();
            onStep = !onStep;
        }
        double targetDeg = STEP_MODE && onStep ? STEP_TARGET_DEG : TARGET_DEG;
        turret.updateTurretToAngle(Math.toRadians(targetDeg), 0);

        telemetry.addData("State", turret.getState());
        telemetry.addData("Target (deg)", targetDeg);
        telemetry.addData("Angle (deg)", Math.toDegrees(turret.getAngle()));
        telemetry.addData("Error (deg)", Math.toDegrees(turret.getError()));
        telemetry.addData("Power", turret.getPower());
        telemetry.addData("On target", turret.goodTurretConfig());
        telemetry.addData("Gains", "kP %.3f  kD %.3f  kS %.3f", RobotConfig.TURRET_KP, RobotConfig.TURRET_KD, RobotConfig.TURRET_KS);
        telemetry.addLine("A = re-home");
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
