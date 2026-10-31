package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry;
import org.firstinspires.ftc.teamcode.robot.helperClasses.FieldGeometry.Cell;
import org.firstinspires.ftc.teamcode.robot.helperClasses.HiveTracker;

/**
 * Test screen for the Limelight AprilTags: which tags it sees, distance and bearing to our up
 * cell, and the HIVE tip state. Needs only the Limelight in the config.
 *
 * <pre>
 * X      switch alliance (init or running)
 * start  "the HIVE tipped" (driver override)
 * </pre>
 */
@Configurable
@TeleOp(name = "TagVisionTest", group = "Test")
public class TagVisionTest extends OpMode {
    private final RobotHardware robot = new RobotHardware();
    private final Vision vision = new Vision(robot);
    private final ElapsedTime clock = new ElapsedTime();
    private TelemetryManager panelsTelemetry;
    private HiveTracker hive;
    private boolean red = true;

    @Override
    public void init() {
        robot.initVision(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(TagVisionTest.class);
        vision.start();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            red = !red;
        }
        panelsTelemetry.addData("Alliance (X)", red ? "RED, tags 30-37" : "BLUE, tags 38-45");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        hive = new HiveTracker(FieldGeometry.startingUpCell(red));
        clock.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            red = !red;
            hive = new HiveTracker(FieldGeometry.startingUpCell(red));
        }
        if (gamepad1.startWasPressed()) {
            hive.flip();
        }
        Cell up = hive.getUpCell();
        vision.setTargetCluster(FieldGeometry.tagBase(red, up), FieldGeometry.tagBase(red, FieldGeometry.other(up)));
        vision.update(0.0, 0.0); // robot frame: bearing is relative to the camera
        hive.update(vision.hasTarget(), vision.seesOtherCell(), clock.seconds());

        panelsTelemetry.addData("Alliance (X)", red ? "RED" : "BLUE");
        panelsTelemetry.addData("Up cell", String.format("%s, tips %d", hive.getUpCell(), hive.getTips()));
        panelsTelemetry.addData("Tags seen", vision.hasTarget() ? vision.getVisibleIds() : "none");
        panelsTelemetry.addData("Other cell in view", vision.seesOtherCell() ? "YES (tipped?)" : "-");
        panelsTelemetry.addData("Distance (in)", String.format("%.1f", vision.getDistanceInches()));
        panelsTelemetry.addData("Bearing (deg, + right)", String.format("%.1f", vision.getBearingDeg()));
        panelsTelemetry.addData("Avg ta (%)", String.format("%.3f", vision.getAverageTa()));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        vision.stop();
    }
}
