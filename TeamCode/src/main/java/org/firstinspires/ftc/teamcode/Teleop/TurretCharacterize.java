package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.turret;

/**
 * Measures steady-state turret speed at a list of powers.
 * All hardware setup (servo directions, encoder sign, PWM range) comes from turret.init(),
 * so this test always matches the real robot code.
 */
@TeleOp(name = "Turret Characterize", group = "Test")
public class TurretCharacterize extends LinearOpMode {

    // Safety limit: cut power if the turret gets this far from where it started (rad).
    // SET THIS below your real wiring limit. 2.0 rad ≈ 115°.
    static final double LIMIT_RAD = 5000.0;

    static final double SETTLE_S = 1.5;   // let speed stabilize
    static final double MEASURE_S = 1;    // measure over this window

    // true  = fine sweep 0.05..0.5 to find p_sat (where speed stops increasing)
    // false = coarse sweep 0.2..1.0 for fitting kS/kV after PWM range is set
    static final boolean FINE_SWEEP = false;

    static final double[] FINE = {0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2, 0.25, -0.25,
                                  0.3, -0.3, 0.35, -0.35, 0.4, -0.4, 0.45, -0.45, 0.5, -0.5};
    static final double[] COARSE = {0.2, -0.2, 0.35, -0.35, 0.5, -0.5, 0.65, -0.65, 0.8, -0.8, 1, -1};

    turret t = new turret();

    @Override
    public void runOpMode() {
        // ---------- 1. INIT ----------
        t.init(this);   // directions, encoder sign, PWM range, encoder reset
        double[] powers = FINE_SWEEP ? FINE : COARSE;

        telemetry.addLine("Center the turret, then press START");
        telemetry.addData("Sweep", FINE_SWEEP ? "FINE (find p_sat)" : "COARSE (fit kS/kV)");
        telemetry.addData("PWM range", "%.0f - %.0f us", turret.PWM_LOW, turret.PWM_HIGH);
        telemetry.update();

        // ---------- 2. WAIT ----------
        waitForStart();

        // ---------- 3. RUN ----------
        double[] results = new double[powers.length];
        boolean[] hitLimit = new boolean[powers.length];

        for (int i = 0; i < powers.length && opModeIsActive(); i++) {
            double power = powers[i];
            t.setPower(power);

            ElapsedTime timer = new ElapsedTime();
            boolean measuring = false;
            double startAngle = 0, startTime = 0;
            double endAngle = 0, endTime = 0;

            while (opModeIsActive() && timer.seconds() < SETTLE_S + MEASURE_S) {
                t.updateVelocity();
                double angle = t.turretAngle();

                if (Math.abs(angle) > LIMIT_RAD) {   // safety first
                    hitLimit[i] = true;
                    break;
                }
                // Average speed = total angle moved / time. Uses 32-bit position,
                // so it can't overflow like the hub's 16-bit velocity.
                if (timer.seconds() > SETTLE_S) {
                    if (!measuring) {
                        measuring = true;
                        startAngle = angle;
                        startTime = timer.seconds();
                    }
                    endAngle = angle;
                    endTime = timer.seconds();
                }
                telemetry.addData("Run", "%d / %d  power %.2f", i + 1, powers.length, power);
                telemetry.addData("Angle (deg)", "%.1f", Math.toDegrees(angle));
                telemetry.addData("Vel (rad/s)", "%.2f", t.tVelocity());
                telemetry.update();
            }

            t.setPower(0);
            double dt = endTime - startTime;
            results[i] = (measuring && dt > 0) ? (endAngle - startAngle) / dt : Double.NaN;
            sleep(500); // let it stop before reversing
        }

        t.setPower(0);

        // ---------- 4. SHOW RESULTS ----------
        while (opModeIsActive()) {
            for (int i = 0; i < powers.length; i++) {
                telemetry.addData(String.format("power %+.2f", powers[i]),
                        hitLimit[i] ? "HIT LIMIT (shorten SETTLE/MEASURE or lower power)"
                                : String.format("%.3f rad/s", results[i]));
            }
            telemetry.update();
        }
    }
}
