package org.firstinspires.ftc.teamcode.robot.helperClasses;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes a CSV of the match (pose, flywheel, turret, storage...) to the Control Hub so you can
 * graph what happened afterwards. Pull the files with Android Studio's Device Explorer or
 * {@code adb pull /sdcard/FIRST/data/matchlogs}.
 *
 * <p>Usage, once per loop: {@code if (log.shouldLog(t)) { log.value(x).value(y).endRow(); }}</p>
 *
 * <p>Logging can never crash the OpMode: any I/O error just turns the logger off.
 * Rows go into one reused StringBuilder and a buffered writer, and are throttled
 * to {@link #PERIOD}.</p>
 */
public class MatchLogger {
    public static double PERIOD = 0.05;     // seconds between rows (20 Hz)
    public static boolean ENABLED = true;

    private BufferedWriter writer;
    private final StringBuilder row = new StringBuilder(256);
    private final String path;
    private double lastLogTime = -1;
    private boolean rowOpen = false;
    private double lastFlushTime = -1;
    /** Push buffered rows to disk this often, so a crash or power cut loses at most ~1 s. */
    public static double FLUSH_PERIOD = 1.0;

    /**
     * @param dir     folder for logs (AppUtil.ROBOT_DATA_DIR/matchlogs on the robot)
     * @param name    file name prefix, e.g. the OpMode name
     * @param columns CSV header
     */
    public MatchLogger(File dir, String name, String... columns) {
        String file = null;
        if (ENABLED) {
            try {
                if (!dir.exists() && !dir.mkdirs()) {
                    throw new IOException("can't create " + dir);
                }
                String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
                File out = new File(dir, name + "_" + stamp + ".csv");
                writer = new BufferedWriter(new FileWriter(out), 1 << 16);
                file = out.getAbsolutePath();
                writer.write("t");
                for (String column : columns) {
                    writer.write(',');
                    writer.write(column);
                }
                writer.newLine();
            } catch (IOException e) {
                writer = null;
            }
        }
        path = file;
    }

    /** True if it's time for another row (and logging is working). Starts a new row. */
    public boolean shouldLog(double time) {
        if (writer == null || time - lastLogTime < PERIOD) {
            return false;
        }
        lastLogTime = time;
        row.setLength(0);
        row.append(Math.round(time * 1000) / 1000.0);
        rowOpen = true;
        return true;
    }

    public MatchLogger value(double v) {
        row.append(',');
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            row.append("NaN"); // Math.round would turn these into a believable 0 / huge number
            return this;
        }
        // 3 decimals is plenty and keeps files small
        row.append(Math.round(v * 1000) / 1000.0);
        return this;
    }

    public MatchLogger value(boolean v) {
        row.append(',').append(v ? 1 : 0);
        return this;
    }

    public MatchLogger value(int v) {
        row.append(',').append(v);
        return this;
    }

    public void endRow() {
        if (writer == null || !rowOpen) {
            return; // only write rows that shouldLog() started
        }
        rowOpen = false;
        try {
            writer.append(row);
            writer.newLine();
            if (lastLogTime - lastFlushTime >= FLUSH_PERIOD) {
                lastFlushTime = lastLogTime;
                writer.flush();
            }
        } catch (IOException e) {
            close();
        }
    }

    /** Call in stop(). Flushes everything to disk. */
    public void close() {
        if (writer == null) {
            return;
        }
        try {
            writer.close();
        } catch (IOException ignored) {
            // nothing useful to do at the end of a match
        }
        writer = null;
    }

    public boolean isLogging() {
        return writer != null;
    }

    /** Full path of the log file, or null if logging is off. */
    public String getPath() {
        return path;
    }
}
