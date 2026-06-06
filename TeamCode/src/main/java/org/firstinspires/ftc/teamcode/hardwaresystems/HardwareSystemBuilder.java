package org.firstinspires.ftc.teamcode.hardwaresystems;

/**
 * Builder for complex hardware systems. The states of the builder should be
 * stored in classes that implement {@link BuilderParameters}. Ideally, multiple
 * {@link BuilderParameters} should be used to group together related
 * attributes/settings.
 * <p>
 * The builder should have setter methods for each attribute in each of the
 * {@link BuilderParameters} subclasses, and each of the setters should return
 * {@code this} so that they can be chained, e.g,
 * <pre>
 * {@code
 * Example example = new Builder().setA()
 *                                .setB()
 *                                .setC()
 *                                .build();
 * }
 * </pre>
 */
public abstract class HardwareSystemBuilder {
    /**
     * Instantiate any private attributes that may need to be instantiated. They
     * should be instantiated to an invalid or default value, depending on the
     * behavior desired. If the parameter is required, set it to an invalid
     * value. Otherwise, set it to a reasonable default.
     */
    public HardwareSystemBuilder() {
    }

    /**
     * Return whether the current values stored for the parameters are valid,
     * such that the builder can create a meaningful object (e.g., the motors
     * are not {@code null} or the power is negative).
     *
     * @return Whether the current values stored for the parameters are valid,
     * such that the builder can create a meaningful object (e.g., the motors
     * are not {@code null} or the power is negative).
     */
    public abstract boolean isValid();

    /**
     * Instantiate a <em>specific</em> type of hardware system (i.e., one of the
     * classes in {@code hardwaresystems}), rather than a general
     * {@link Object}, using the given {@link BuilderParameters}.
     *
     * @return A <em>specific</em> type of hardware system (i.e., one of the
     * classes in {@code hardwaresystems}), rather than a general
     * {@link Object}.
     * <p>
     * If the {@link BuilderParameters} are invalid (as defined by
     * {@link BuilderParameters#isValid()}), fail and return {@code null}.
     */
    public abstract Object build();
}