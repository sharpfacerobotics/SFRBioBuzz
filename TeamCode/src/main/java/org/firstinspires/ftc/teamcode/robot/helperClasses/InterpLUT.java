package com.arcrobotics.ftclib.util;

import java.util.Arrays;

/**
 * Shape-preserving piecewise cubic Hermite interpolation (PCHIP).
 *
 * <p>Provide all control points at construction time. X values must be finite
 * and strictly increasing; Y values must be finite. Inputs outside the domain
 * clamp to the nearest endpoint. NaN input returns NaN.</p>
 *
 * <p>Each interval is converted once into a cubic polynomial in normalized
 * coordinates t in [0, 1]:</p>
 *
 * <pre>
 * t = (x - x0) / (x1 - x0)
 * P(t) = a*t^3 + b*t^2 + c*t + d
 * </pre>
 *
 * <p>The four coefficients for every interval are precomputed in the
 * constructor. Runtime lookup is therefore only a binary search, normalization,
 * and a cubic evaluation using Horner's method.</p>
 */
public final class InterpLUT {

    private final double[] mX;
    private final double[] mY;

    // Four coefficients per interval: [a, b, c, d].
    // Interval i begins at index 4*i.
    private final double[] mCoefficients;

    /**
     * Constructs and fully builds the interpolation table.
     *
     * @param x strictly increasing X coordinates
     * @param y Y coordinates corresponding one-to-one with x
     */
    public InterpLUT(double[] x, double[] y) {
        validateInput(x, y);

        // Defensive copies prevent the caller from silently changing the LUT
        // after its polynomial coefficients have already been calculated.
        mX = x.clone();
        mY = y.clone();
        mCoefficients = buildCoefficients(mX, mY);
    }

    private static void validateInput(double[] x, double[] y) {
        if (x == null || y == null) {
            throw new IllegalArgumentException("X and Y arrays cannot be null.");
        }
        if (x.length != y.length) {
            throw new IllegalArgumentException("X and Y arrays must have equal length.");
        }
        if (x.length < 2) {
            throw new IllegalArgumentException("At least two control points are required.");
        }

        for (int i = 0; i < x.length; i++) {
            if (!Double.isFinite(x[i]) || !Double.isFinite(y[i])) {
                throw new IllegalArgumentException(
                        "Control-point X and Y values must be finite at index " + i + ".");
            }
            if (i > 0 && !(x[i] > x[i - 1])) {
                throw new IllegalArgumentException(
                        "X values must be strictly increasing at index " + i + ".");
            }
        }
    }

    private static double[] buildCoefficients(double[] x, double[] y) {
        final int n = x.length;

        double[] widths = new double[n - 1];
        double[] secants = new double[n - 1];
        double[] tangents = new double[n];

        // Secant slope of every interval.
        for (int i = 0; i < n - 1; i++) {
            double h = x[i + 1] - x[i];
            double d = (y[i + 1] - y[i]) / h;

            if (!Double.isFinite(d)) {
                throw new IllegalArgumentException(
                        "Control-point scale produced a non-finite slope at interval " + i + ".");
            }

            widths[i] = h;
            secants[i] = d;
        }

        if (n == 2) {
            // With only two points, the unique shape-preserving interpolation
            // is simply the straight line joining them.
            tangents[0] = secants[0];
            tangents[1] = secants[0];
        } else {
            // Interior PCHIP tangents.
            // Same-sign neighboring secants use a weighted harmonic mean.
            // Flat sections or direction changes get zero tangent to avoid
            // creating a new peak/valley between control points.
            for (int i = 1; i < n - 1; i++) {
                double left = secants[i - 1];
                double right = secants[i];

                if (left == 0.0 || right == 0.0
                        || Math.signum(left) != Math.signum(right)) {
                    tangents[i] = 0.0;
                } else {
                    double w1 = 2.0 * widths[i] + widths[i - 1];
                    double w2 = widths[i] + 2.0 * widths[i - 1];
                    tangents[i] = (w1 + w2) / (w1 / left + w2 / right);
                }
            }

            tangents[0] = endpointTangent(
                    widths[0], widths[1], secants[0], secants[1]);
            tangents[n - 1] = endpointTangent(
                    widths[n - 2], widths[n - 3], secants[n - 2], secants[n - 3]);
        }

        // Convert each interval into P(t) = a*t^3 + b*t^2 + c*t + d,
        // where t = (x - x0) / h.
        double[] coefficients = new double[4 * (n - 1)];

        for (int i = 0; i < n - 1; i++) {
            double y0 = y[i];
            double y1 = y[i + 1];
            double h = widths[i];

            // Because t is normalized, dP/dt = h * dP/dx.
            double m0 = h * tangents[i];
            double m1 = h * tangents[i + 1];

            int k = 4 * i;
            coefficients[k]     =  2.0 * y0 - 2.0 * y1 + m0 + m1;       // a
            coefficients[k + 1] = -3.0 * y0 + 3.0 * y1 - 2.0 * m0 - m1; // b
            coefficients[k + 2] = m0;                                    // c
            coefficients[k + 3] = y0;                                    // d

            for (int j = 0; j < 4; j++) {
                if (!Double.isFinite(coefficients[k + j])) {
                    throw new IllegalArgumentException(
                            "Control-point scale produced a non-finite coefficient at interval " + i + ".");
                }
            }
        }

        return coefficients;
    }

    /**
     * Shape-preserving one-sided endpoint tangent used by PCHIP.
     */
    private static double endpointTangent(double h0, double h1, double d0, double d1) {
        double result = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);

        if (Math.signum(result) != Math.signum(d0)) {
            return 0.0;
        }

        if (Math.signum(d0) != Math.signum(d1)
                && Math.abs(result) > 3.0 * Math.abs(d0)) {
            return 3.0 * d0;
        }

        return result;
    }

    /**
     * Evaluates the interpolation at the requested X value.
     * Values outside the LUT domain clamp to the nearest endpoint.
     */
    public double get(double input) {
        if (Double.isNaN(input)) {
            return Double.NaN;
        }

        final int last = mX.length - 1;

        if (input <= mX[0]) {
            return mY[0];
        }
        if (input >= mX[last]) {
            return mY[last];
        }

        // Arrays.binarySearch returns the exact index when input is a control
        // point, otherwise -(insertionPoint) - 1.
        int index = Arrays.binarySearch(mX, input);
        if (index >= 0) {
            return mY[index];
        }

        int i = -index - 2;
        double t = (input - mX[i]) / (mX[i + 1] - mX[i]);
        int k = 4 * i;

        // Horner's method:
        // a*t^3 + b*t^2 + c*t + d
        // = ((a*t + b)*t + c)*t + d
        return ((mCoefficients[k] * t + mCoefficients[k + 1]) * t
                + mCoefficients[k + 2]) * t + mCoefficients[k + 3];
    }

    /** Number of supplied control points. */
    public int size() {
        return mX.length;
    }

    @Override
    public String toString() {
        StringBuilder result = new StringBuilder("[");
        for (int i = 0; i < mX.length; i++) {
            if (i > 0) {
                result.append(", ");
            }
            result.append('(').append(mX[i]).append(", ").append(mY[i]).append(')');
        }
        return result.append(']').toString();
    }
}
