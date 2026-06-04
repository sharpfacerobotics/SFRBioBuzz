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
public interface HardwareSystemBuilder {
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
    Object build();
}