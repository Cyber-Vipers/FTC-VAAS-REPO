package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import static org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.ShotTableConstants.*;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.Constants.LiftConstants;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Distance to the goal (inches) -> lift position (ticks) and flywheel speed (ticks/s), fitted
 * by least squares over shots that went in, recorded with the HoneyComb Shot Tuner.
 *
 * The fit is saved on the Control Hub (FIRST/settings/honeycomb_shot_table.txt), so every
 * OpMode picks it up at Init. Without that file the DEFAULT_* coefficients in
 * Constants.ShotTableConstants are used: paste the tuner's printed lines there so a fresh
 * Control Hub still shoots correctly.
 *
 * Coefficients are lowest order first: value = c[0] + c[1]*d + c[2]*d^2. Distance is clamped to
 * the recorded range so the fit never guesses beyond the data, and the outputs are clamped to
 * the lift's limits and the flywheel's top speed.
 */
public class ShotTable {
    public static final String FILE_NAME = "honeycomb_shot_table.txt";

    public static class DataPoint {
        public final double distance, liftTicks, flywheelTps;

        public DataPoint(double distance, double liftTicks, double flywheelTps) {
            this.distance = distance;
            this.liftTicks = liftTicks;
            this.flywheelTps = flywheelTps;
        }
    }

    public final List<DataPoint> points = new ArrayList<>();
    public double[] liftCoeffs = DEFAULT_LIFT_COEFFS.clone();
    public double[] flywheelCoeffs = DEFAULT_FLYWHEEL_COEFFS.clone();
    public double minDist = DEFAULT_MIN_DIST_IN, maxDist = DEFAULT_MAX_DIST_IN;
    public double liftR2 = Double.NaN, flywheelR2 = Double.NaN;
    public boolean loadedFromFile = false;

    public double liftTicks(double distance) {
        return Range.clip(eval(liftCoeffs, Range.clip(distance, minDist, maxDist)),
                LiftConstants.LOWEST_TICKS, LiftConstants.HIGHEST_TICKS);
    }

    public double flywheelTps(double distance) {
        return Range.clip(eval(flywheelCoeffs, Range.clip(distance, minDist, maxDist)),
                0, FlywheelConstants.MAX_TPS);
    }

    /** Refit both curves from the points. The degree drops when there are too few distances. */
    public boolean fit() {
        int n = points.size();
        if (n == 0) return false;
        // A degree-k curve needs k+1 different distances; repeat shots from one spot don't count.
        int degree = Math.min(MAX_DEGREE, distinctDistances() - 1);

        double[] d = new double[n], lift = new double[n], speed = new double[n];
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            DataPoint p = points.get(i);
            d[i] = p.distance;
            lift[i] = p.liftTicks;
            speed[i] = p.flywheelTps;
            lo = Math.min(lo, p.distance);
            hi = Math.max(hi, p.distance);
        }

        double[] newLift = null, newSpeed = null;
        for (int deg = degree; deg >= 0 && (newLift == null || newSpeed == null); deg--) {
            newLift = polyFit(d, lift, deg);
            newSpeed = polyFit(d, speed, deg);
        }
        if (newLift == null || newSpeed == null) return false;
        liftCoeffs = newLift;
        flywheelCoeffs = newSpeed;
        minDist = lo;
        maxDist = hi;
        liftR2 = rSquared(d, lift, liftCoeffs);
        flywheelR2 = rSquared(d, speed, flywheelCoeffs);
        return true;
    }

    /** Number of recorded distances more than 1 inch apart from each other. */
    private int distinctDistances() {
        List<Double> seen = new ArrayList<>();
        for (DataPoint p : points) {
            boolean dup = false;
            for (double d : seen) dup |= Math.abs(d - p.distance) < 1;
            if (!dup) seen.add(p.distance);
        }
        return seen.size();
    }

    // ---- persistence -----------------------------------------------------------------------

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public void save() {
        StringBuilder sb = new StringBuilder();
        sb.append("lift=").append(join(liftCoeffs)).append('\n');
        sb.append("flywheel=").append(join(flywheelCoeffs)).append('\n');
        sb.append("range=").append(minDist).append(',').append(maxDist).append('\n');
        for (DataPoint p : points) {
            sb.append("point=").append(p.distance).append(',').append(p.liftTicks)
                    .append(',').append(p.flywheelTps).append('\n');
        }
        ReadWriteFile.writeFile(file(), sb.toString());
    }

    /** Loads the saved fit and its points if the file exists; otherwise keeps the defaults. */
    public static ShotTable load() {
        ShotTable t = new ShotTable();
        File f = file();
        if (!f.exists()) return t;
        try {
            for (String line : ReadWriteFile.readFileOrThrow(f).split("\n")) {
                String[] kv = line.trim().split("=", 2);
                if (kv.length != 2) continue;
                double[] v = parse(kv[1]);
                switch (kv[0]) {
                    case "lift": t.liftCoeffs = v; break;
                    case "flywheel": t.flywheelCoeffs = v; break;
                    case "range": t.minDist = v[0]; t.maxDist = v[1]; break;
                    case "point": t.points.add(new DataPoint(v[0], v[1], v[2])); break;
                }
            }
            t.loadedFromFile = true;
        } catch (Exception e) {
            return new ShotTable(); // unreadable file: fall back to the defaults
        }
        return t;
    }

    /** Java lines for Constants.ShotTableConstants, to paste back in. */
    public String coefficientsAsJava() {
        return String.format(Locale.US,
                "DEFAULT_LIFT_COEFFS = {%s};\nDEFAULT_FLYWHEEL_COEFFS = {%s};\nDEFAULT_MIN_DIST_IN = %.1f, DEFAULT_MAX_DIST_IN = %.1f;",
                join(liftCoeffs), join(flywheelCoeffs), minDist, maxDist);
    }

    // ---- math ------------------------------------------------------------------------------

    static double eval(double[] c, double x) {
        double y = 0;
        for (int i = c.length - 1; i >= 0; i--) y = y * x + c[i];
        return y;
    }

    /** Least-squares polynomial fit via the normal equations. Null if it can't be solved. */
    static double[] polyFit(double[] x, double[] y, int degree) {
        int m = degree + 1;
        double[][] a = new double[m][m + 1];
        for (int i = 0; i < x.length; i++) {
            double[] pow = new double[2 * m - 1];
            pow[0] = 1;
            for (int k = 1; k < pow.length; k++) pow[k] = pow[k - 1] * x[i];
            for (int r = 0; r < m; r++) {
                for (int c = 0; c < m; c++) a[r][c] += pow[r + c];
                a[r][m] += pow[r] * y[i];
            }
        }
        return solve(a);
    }

    /** Gaussian elimination with partial pivoting on an augmented matrix. */
    private static double[] solve(double[][] a) {
        int m = a.length;
        for (int col = 0; col < m; col++) {
            int pivot = col;
            for (int r = col + 1; r < m; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            }
            if (Math.abs(a[pivot][col]) < 1e-12) return null;
            double[] tmp = a[col]; a[col] = a[pivot]; a[pivot] = tmp;
            for (int r = 0; r < m; r++) {
                if (r == col) continue;
                double f = a[r][col] / a[col][col];
                for (int c = col; c <= m; c++) a[r][c] -= f * a[col][c];
            }
        }
        double[] out = new double[m];
        for (int i = 0; i < m; i++) out[i] = a[i][m] / a[i][i];
        return out;
    }

    static double rSquared(double[] x, double[] y, double[] c) {
        double mean = 0;
        for (double v : y) mean += v;
        mean /= y.length;
        double ssRes = 0, ssTot = 0;
        for (int i = 0; i < x.length; i++) {
            ssRes += Math.pow(y[i] - eval(c, x[i]), 2);
            ssTot += Math.pow(y[i] - mean, 2);
        }
        return ssTot < 1e-9 ? 1 : 1 - ssRes / ssTot;
    }

    static String join(double[] v) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < v.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format(Locale.US, "%.8g", v[i]));
        }
        return sb.toString();
    }

    private static double[] parse(String s) {
        String[] parts = s.split(",");
        double[] v = new double[parts.length];
        for (int i = 0; i < parts.length; i++) v[i] = Double.parseDouble(parts[i].trim());
        return v;
    }
}
