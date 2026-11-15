package org.firstinspires.ftc.teamcode.subSystems;

/**
 * Shot table: distance (in, turret pivot -> goal, same number Shot Table Tuner shows)
 *   -> hood position, flywheel RPM, time of flight (s).
 *
 * FILL THESE IN from Shot Table Tuner rows + slow-mo TOF. Rows MUST be sorted by distance.
 * The numbers below are PLACEHOLDERS so the code runs - they are NOT tuned.
 *
 * Lookup: straight lines between rows (USE_FIT = false), or a least-squares quadratic
 * through all rows (USE_FIT = true, needs >= 4 rows) which averages out per-row tuning noise.
 * Outside the measured range it CLAMPS to the end rows (never guesses beyond your data).
 */
@com.bylazar.configurables.annotations.Configurable
public class ShotTable {
    public static boolean USE_FIT = false;
    // true = use the CLOSEST row as-is (only shots you actually tested, no blending between rows).
    // Good when neighbouring rows use different hood/rpm combos. Rows every ~3-4 in -> never >2 in from a tested row.
    public static boolean USE_NEAREST = false;

    //                         distance (in)   hood        rpm      tof (s)
    // sorted short -> long (lookup needs this order)
    public static double[] D    = { 21.002, 25.436,  28.48423, 31.227,   35.397,  38, 41.1, 45, 48.6, 54.78, 69};
    public static double[] HOOD = { 0.28,       0.28, 0.12,    0.21,   0.09,  0.11,  0.11, 0.07, 0.1, 0.09, 0.07 };
    public static double[] RPM  = { 2025,       2000, 2100,     2050,  2100,  2090,  2100, 2100, 2138, 2160, 2300 };
    public static double[] TOF  = { 0.46,       0.47, 0.33,    0.56,   0.55,  00.59,  0.47, 0.68, 0.49,  0.67, 0.68};   // measured (only used if SOTM on)

    // Optional: fix the hood when the flywheel is slow (e.g. 4th ball of a burst). Hood units per
    // RPM of error (target - actual). 0 = off. Measure: at one distance, drop rpm 100, find the hood
    // that scores again -> HOOD_PER_RPM = (that hood - normal hood) / 100.
    public static double HOOD_PER_RPM = 0;
    public static double HOOD_COMP_MAX_RPM = 450;   // ignore errors bigger than this (spin-up)

    public static double hood(double d) { return lookup(d, HOOD); }
    public static double rpm(double d)  { return lookup(d, RPM); }
    public static double tof(double d)  { return lookup(d, TOF); }

    /** Hood with the slow-flywheel correction. rpmError = target - actual (+ = too slow). */
    public static double hood(double d, double rpmError) {
        double e = Math.abs(rpmError) > HOOD_COMP_MAX_RPM ? 0 : rpmError;
        return Math.max(0, Math.min(1, hood(d) + HOOD_PER_RPM * e));
    }

    private static double lookup(double d, double[] y) {
        int n = Math.min(D.length, y.length);   // safety: a row missing from one column can't crash it
        double dc = Math.max(D[0], Math.min(D[n - 1], d));   // clamp to measured range
        if (USE_NEAREST) {
            int best = 0;
            for (int i = 1; i < n; i++) if (Math.abs(D[i] - dc) < Math.abs(D[best] - dc)) best = i;
            return y[best];
        }
        if (USE_FIT && n >= 4) {
            double[] c = fitQuadratic(java.util.Arrays.copyOf(D, n), java.util.Arrays.copyOf(y, n));
            return c[0] + c[1] * dc + c[2] * dc * dc;
        }
        for (int i = 0; i < n - 1; i++) {
            if (dc <= D[i + 1]) {
                double t = (dc - D[i]) / (D[i + 1] - D[i]);   // same t for hood, rpm, tof
                return y[i] + t * (y[i + 1] - y[i]);
            }
        }
        return y[n - 1];
    }

    /** Least-squares y = c0 + c1 x + c2 x^2 (3x3 normal equations, Cramer's rule). */
    static double[] fitQuadratic(double[] x, double[] y) {
        double s0 = x.length, s1 = 0, s2 = 0, s3 = 0, s4 = 0, t0 = 0, t1 = 0, t2 = 0;
        for (int i = 0; i < x.length; i++) {
            double xi = x[i], x2 = xi * xi;
            s1 += xi; s2 += x2; s3 += x2 * xi; s4 += x2 * x2;
            t0 += y[i]; t1 += xi * y[i]; t2 += x2 * y[i];
        }
        double det = det3(s0, s1, s2, s1, s2, s3, s2, s3, s4);
        double c0 = det3(t0, s1, s2, t1, s2, s3, t2, s3, s4) / det;
        double c1 = det3(s0, t0, s2, s1, t1, s3, s2, t2, s4) / det;
        double c2 = det3(s0, s1, t0, s1, s2, t1, s2, s3, t2) / det;
        return new double[]{c0, c1, c2};
    }

    private static double det3(double a, double b, double c, double d, double e, double f,
                               double g, double h, double i) {
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    }
}
