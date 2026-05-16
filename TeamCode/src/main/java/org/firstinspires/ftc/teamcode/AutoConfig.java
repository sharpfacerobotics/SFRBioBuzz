package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.stream.Collectors;

/**
 * Run to set the robot's alliance color and side before running autonomous.
 */
@TeleOp(name = "AutoConfig")
public class AutoConfig extends OpMode {
    /**
     * Store whether the robot is on the red or blue alliance.
     */
    public enum AllianceColor {
        RED,
        BLUE
    }

    /**
     * Store whether the robot is on the far or near side.
     */
    public enum TeamSide {
        FAR,
        NEAR
    }

    /**
     * The directory that all the files are saved to.
     */
    private static final String DIRECTORY = Environment.getExternalStorageDirectory().getAbsolutePath() + "/ftc/";
    /**
     * Name of the config file, which is inside the directory specified by {@link #DIRECTORY}.
     */
    private static final String CONFIG_FILE = DIRECTORY + "config.txt";

    /**
     * Return the `String` stored in {@link #CONFIG_FILE}.
     *
     * @return The `String` stored in {@link #CONFIG_FILE}.
     */
    public static String getConfigFile() {
        return CONFIG_FILE;
    }

    /**
     * Read the contents of {@link #CONFIG_FILE} as a {@link String} and return it. Print an error message if it fails.
     *
     * @return The contents of {@link #CONFIG_FILE} as a {@link String}
     */
    public String readAutoConfig() {
        try (BufferedReader buffer = new BufferedReader(new FileReader(CONFIG_FILE))) {
            // Read all the lines joined by newlines.
            return String.join("\n", buffer.lines().collect(Collectors.joining()));

        } catch (IOException e) {
            telemetry.addLine("ERROR: FAILED TO READ AUTO CONFIG FROM " + CONFIG_FILE);
            return null;
        }
    }

    public void parseAutoConfig() {
        String fileData = readAutoConfig();
        AllianceColor ALLIANCE_COLOR =
            (fileData != null)
                ? AllianceColor.valueOf(fileData.split(",")[0])
                : AllianceColor.RED;
        TeamSide TEAM_SIDE = (fileData != null) ? TeamSide.valueOf(fileData.split(",")[1]) : TeamSide.NEAR;
    }

    /**
     * Listen to user input and write the appropriate enum names to {@link #CONFIG_FILE}.
     */
    public void writeConfigFile() {
        String positionString = null;
        if (gamepad1.y || gamepad2.y) {
            // Orange button
            positionString = AllianceColor.RED.name() + "," + TeamSide.NEAR.name();

        } else if (gamepad1.b || gamepad2.b) {
            // Red button
            positionString = AllianceColor.RED.name() + "," + TeamSide.FAR.name();

        } else if (gamepad1.a || gamepad2.a) {
            // Green button
            positionString = AllianceColor.BLUE.name() + "," + TeamSide.NEAR.name();

        } else if (gamepad1.x || gamepad2.x) {
            // Blue button
            positionString = AllianceColor.BLUE.name() + "," + TeamSide.FAR.name();
        }

        // Do nothing if the driver didn't press any buttons.
        if (positionString == null) {
            return;
        }

        // Write the string to the file.
        try (FileWriter writer = new FileWriter(CONFIG_FILE, false)) {
            writer.write(positionString);

        } catch (IOException e) {
            telemetry.addLine("ERROR: FAILED TO WRITE ROBOT CONFIG TO " + CONFIG_FILE);
            telemetry.addLine(e.toString());
        }
    }

    @Override
    public void init() {
        readAutoConfig();
    }

    @Override
    public void loop() {
        telemetry.update();
        String fileData = readAutoConfig();
        telemetry.addData("Current position: ", fileData);

        // If the directory and file do not exist, create them.
        try {
            File directory = new File(DIRECTORY);
            if (!directory.mkdirs()) {
                throw new IOException(DIRECTORY + "could not be created.");
            }

            File file = new File(CONFIG_FILE);
            if (!file.createNewFile()) {
                throw new IOException(CONFIG_FILE + " could not be created.");
            }

        } catch (IOException e) {
            telemetry.addLine(e.getMessage());
        }

        writeConfigFile();
    }
}
