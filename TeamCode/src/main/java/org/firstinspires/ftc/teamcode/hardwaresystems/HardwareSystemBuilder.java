package org.firstinspires.ftc.teamcode.hardwaresystems;

/**
 * Builder for complex hardware systems. The states of the builder should be
 * stored in classes that implement {@link BuilderParameters}.
 */
public interface HardwareSystemBuilder {
    /**
     * Instantiate a <em>specific</em> type of hardware system (i.e., one of the
     * classes in {@code hardwaresystems}), rather than a general
     * {@link Object}.
     *
     * @return A <em>specific</em> type of hardware system (i.e., one of the
     * classes in {@code hardwaresystems}), rather than a general
     * {@link Object}.
     */
    Object build();
}