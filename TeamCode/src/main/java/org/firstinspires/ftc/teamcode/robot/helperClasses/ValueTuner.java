package org.firstinspires.ftc.teamcode.robot.helperClasses;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ValueTuner {
    private double value;
    private final String label;
    private final String buttonHint;

    // Step size configuration built right into the tuner
    private final double[] stepSizes;
    private int stepIndex = 0;

    public ValueTuner(double initialValue, String label, String buttonHint, double[] stepSizes) {
        this.value = initialValue;
        this.label = label;
        this.buttonHint = buttonHint;
        this.stepSizes = stepSizes;
    }

    /**
     * Updates the value and cycles the step size based on native button presses.
     */
    public void update(boolean increase, boolean decrease, boolean cycleStep) {
        if (cycleStep) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        double currentStep = stepSizes[stepIndex];
        if (increase) {
            value += currentStep;
        }
        if (decrease) {
            value -= currentStep;
        }
    }

    public double getValue() {
        return value;
    }

    public double getCurrentStepSize() {
        return stepSizes[stepIndex];
    }

    /**
     * Prints telemetry specifying the variable, its button hints, current value, and active step size.
     */
    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData(label, "%.5f (%s) [Step: %.5f]", value, buttonHint, getCurrentStepSize());
    }
}