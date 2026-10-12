package org.firstinspires.ftc.teamcode.robot.helperClasses;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Draws the robot on the Panels field view: position, heading, and the line the turret is
 * aiming along. Throttled so it doesn't eat loop time.
 */
public class FieldDrawing {
    public static double ROBOT_RADIUS = 9.0;      // inches
    public static double UPDATE_PERIOD = 0.1;     // seconds (10 Hz is plenty to watch)

    private static final String ROBOT_COLOR = "#3F51B5";
    private static final String AIM_COLOR = "#FF9800";
    private static final String GOAL_COLOR = "#4CAF50";

    private final FieldManager field = PanelsField.INSTANCE.getField();
    private final ElapsedTime timer = new ElapsedTime();

    public FieldDrawing() {
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * @param x            robot x (in, Pedro)
     * @param y            robot y
     * @param heading      robot heading (rad)
     * @param aimHeading   field heading the shot is aimed along (rad)
     * @param aimDistance  how long to draw the aim line (in)
     * @param goalX        hive x
     * @param goalY        hive y
     */
    public void draw(double x, double y, double heading, double aimHeading, double aimDistance,
                     double goalX, double goalY) {
        if (timer.seconds() < UPDATE_PERIOD) {
            return;
        }
        timer.reset();

        field.setStyle(GOAL_COLOR, GOAL_COLOR, 1);
        field.moveCursor(goalX, goalY);
        field.circle(4);

        field.setStyle("transparent", ROBOT_COLOR, 1.5);
        field.moveCursor(x, y);
        field.circle(ROBOT_RADIUS);
        field.moveCursor(x, y);
        field.line(x + ROBOT_RADIUS * Math.cos(heading), y + ROBOT_RADIUS * Math.sin(heading));

        field.setStyle("transparent", AIM_COLOR, 1);
        field.moveCursor(x, y);
        field.line(x + aimDistance * Math.cos(aimHeading), y + aimDistance * Math.sin(aimHeading));

        field.update();
    }
}
