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
 * Set the robot's initial state and actions before running {@link Auto}.
 */
@TeleOp(name = "AutoConfigurator")
public class AutoConfigurator extends OpMode {
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
    public enum AllianceSide {
        FAR,
        NEAR
    }

    /**
     * Represent the data in {@link #CONFIG_FILE} as enums.
     */
    public static class AutoConfig {
        private final AllianceColor ALLIANCE_COLOR;
        private final AllianceSide ALLIANCE_SIDE;

        public AutoConfig(
            AllianceColor allianceColor,
            AllianceSide allianceSide
        ) {
            this.ALLIANCE_COLOR = allianceColor;
            this.ALLIANCE_SIDE = allianceSide;
        }

        /**
         * Return the alliance color stored in {@link #CONFIG_FILE}.
         *
         * @return The alliance color stored in {@link #CONFIG_FILE}.
         */
        public AllianceColor getAllianceColor() {
            return ALLIANCE_COLOR;
        }

        /**
         * Return the alliance side stored in {@link #CONFIG_FILE}.
         *
         * @return The alliance side stored in {@link #CONFIG_FILE}.
         */
        public AllianceSide getAllianceSide() {
            return ALLIANCE_SIDE;
        }
    }

    /**
     * The directory that all the files are saved to.
     */
    private static final String DIRECTORY =
        Environment.getExternalStorageDirectory()
                   .getAbsolutePath()
        + "/ftc/";

    /**
     * Name of the config file, which is inside the directory specified by
     * {@link #DIRECTORY}.
     */
    private static final String CONFIG_FILE = DIRECTORY + "config.csv";
    /**
     * The separator used in {@link #CONFIG_FILE}.
     */
    private static final String SEPARATOR = ",";

    /**
     * Return the {@link String} stored in {@link #CONFIG_FILE}.
     *
     * @return The {@link String} stored in {@link #CONFIG_FILE}.
     */
    public static String getConfigFile() {
        return CONFIG_FILE;
    }

    /**
     * Read the contents of {@link #CONFIG_FILE} as a {@link String} and return
     * it. Print an error message if it fails.
     *
     * @return The contents of {@link #CONFIG_FILE} as a {@link String}. If an
     * exception occurs, return {@code null}.
     */
    public static String readConfigFile() {
        /*
         * If the file is unreadable for some reason, return null.
         */
        try (BufferedReader buffer = new BufferedReader(new FileReader(
            CONFIG_FILE))) {
            // Read all the lines joined by newlines.
            String fileString = String.join(
                "\n",
                buffer.lines().collect(Collectors.joining())
            ).trim();

            // If the file is invalid, treat it as invalid.
            if (fileString.isEmpty()) {
                throw new IOException(CONFIG_FILE + " is empty.");
            }

            return fileString;

        } catch (IOException e) {
            return null;
        }
    }

    /**
     * Read the data stored in {@link #CONFIG_FILE} and instantiate an
     * {@link AutoConfig} object to store the information.
     *
     * @return An {@link AutoConfig} instance representing the data in
     * {@link #CONFIG_FILE}.
     */
    public static AutoConfig parseConfigFile() {
        String fileString = readConfigFile();
        // Default to red near.
        if (fileString == null) {
            return new AutoConfig(AllianceColor.RED, AllianceSide.NEAR);
        }

        /*
         * Try to split the file to extract the values.
         *
         * If the file has been corrupted or improperly formatted, default to
         *  red near.
         */
        String[] values = fileString.split(SEPARATOR);
        try {
            return new AutoConfig(
                AllianceColor.valueOf(values[0]),
                AllianceSide.valueOf(values[1])
            );

        } catch (IndexOutOfBoundsException exception) {
            return new AutoConfig(AllianceColor.RED, AllianceSide.NEAR);
        }
    }

    /**
     * Listen to user input and write the appropriate enum names to
     * {@link #CONFIG_FILE}.
     */
    public void writeConfigFile() {
        String positionString = null;
        if (gamepad1.y || gamepad2.y) {
            // Orange button
            positionString = AllianceColor.RED.name()
                             + SEPARATOR
                             + AllianceSide.NEAR.name();

        } else if (gamepad1.b || gamepad2.b) {
            // Red button
            positionString = AllianceColor.RED.name()
                             + SEPARATOR
                             + AllianceSide.FAR.name();

        } else if (gamepad1.a || gamepad2.a) {
            // Green button
            positionString = AllianceColor.BLUE.name()
                             + SEPARATOR
                             + AllianceSide.NEAR.name();

        } else if (gamepad1.x || gamepad2.x) {
            // Blue button
            positionString = AllianceColor.BLUE.name()
                             + SEPARATOR
                             + AllianceSide.FAR.name();
        }

        // Do nothing if the driver didn't press any buttons.
        if (positionString == null) {
            return;
        }

        // Write the string to the file.
        try (FileWriter writer = new FileWriter(CONFIG_FILE, false)) {
            writer.write(positionString);

        } catch (IOException e) {
            telemetry.addLine("ERROR: FAILED TO WRITE AUTO CONFIG TO "
                              + CONFIG_FILE);
            telemetry.addLine(e.toString());
        }
    }

    @Override
    public void init() {
        readConfigFile();
    }

    @Override
    public void loop() {
        telemetry.update();
        String fileData = readConfigFile();
        telemetry.addLine(
            (fileData != null) ?
            "Current config: " + fileData :
            "ERROR: FAILED TO READ AUTO CONFIG FROM " + CONFIG_FILE
        );

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
