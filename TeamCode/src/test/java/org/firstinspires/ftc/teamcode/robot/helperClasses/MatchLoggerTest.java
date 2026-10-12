package org.firstinspires.ftc.teamcode.robot.helperClasses;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.List;

public class MatchLoggerTest {
    @Rule
    public TemporaryFolder tmp = new TemporaryFolder();

    @Test
    public void writesHeaderAndThrottledRows() throws Exception {
        MatchLogger log = new MatchLogger(new File(tmp.getRoot(), "logs"), "test", "x", "ready", "n");
        assertTrue(log.isLogging());
        for (int i = 0; i < 10; i++) {
            double t = i * 0.02; // 50 Hz loop, logger at 20 Hz
            if (log.shouldLog(t)) {
                log.value(1.23456).value(true).value(7).endRow();
            }
        }
        log.close();

        List<String> lines = Files.readAllLines(new File(log.getPath()).toPath(), StandardCharsets.UTF_8);
        assertEquals("t,x,ready,n", lines.get(0));
        assertEquals(1 + 4, lines.size()); // rows at t = 0, 0.06, 0.12, 0.18
        assertEquals("0.0,1.235,1,7", lines.get(1));
    }

    @Test
    public void endRowWithoutShouldLogWritesNothing() throws Exception {
        MatchLogger log = new MatchLogger(tmp.getRoot(), "test", "x");
        log.value(5).endRow();
        log.close();
        assertEquals(1, Files.readAllLines(new File(log.getPath()).toPath()).size());
    }

    @Test
    public void badFolderTurnsLoggingOffInsteadOfCrashing() throws Exception {
        File notADir = tmp.newFile("file");
        MatchLogger log = new MatchLogger(new File(notADir, "sub"), "test", "x");
        assertFalse(log.isLogging());
        assertFalse(log.shouldLog(0));
        log.value(1).endRow();
        log.close();
        assertNotNull(log); // still alive
    }
}
