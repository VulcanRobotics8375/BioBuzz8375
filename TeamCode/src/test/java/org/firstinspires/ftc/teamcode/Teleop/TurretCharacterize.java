package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Turret Characterize", group = "Test")
public class TurretCharacterize extends LinearOpMode {

    // 8192 ticks per middle-axle rev, axle turns 96/34 revs per turret rev
    static final double TICKS_PER_TURRET_RAD = 8192.0 * (96.0 / 34.0) / (2 * Math.PI);

    // Safety limit: cut power if the turret gets this far from where it started (rad).
    // SET THIS below your real wiring limit. 2.0 rad ≈ 115°.
    static final double LIMIT_RAD = 2.0;

    static final double SETTLE_S = 0.5;   // let speed stabilize
    static final double MEASURE_S = 0.3;  // average over this window

    // Paired +/- so the turret ends up roughly where it started
    static final double[] POWERS = {0.2, -0.2, 0.35, -0.35, 0.5, -0.5, 0.65, -0.65};

    DcMotorEx encoder;
    CRServo s1, s2;

    @Override
    public void runOpMode() {
        // ---------- 1. INIT: runs when you press INIT ----------
        encoder = hardwareMap.get(DcMotorEx.class, "tEnc");
        s1 = hardwareMap.get(CRServo.class, "tServo1");
        s2 = hardwareMap.get(CRServo.class, "tServo2");
        // s2.setDirection(DcMotorSimple.Direction.REVERSE); // uncomment if mounted mirrored

        // Zero the encoder with the turret centered
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Center the turret, then press START");
        telemetry.update();

        // ---------- 2. WAIT: blocks here until you press START ----------
        waitForStart();

        // ---------- 3. RUN: your test, top to bottom, once ----------
        double[] results = new double[POWERS.length];
        boolean[] hitLimit = new boolean[POWERS.length];

        for (int i = 0; i < POWERS.length && opModeIsActive(); i++) {
            double power = POWERS[i];
            setPower(power);

            ElapsedTime t = new ElapsedTime();
            double sum = 0;
            int n = 0;

            while (opModeIsActive() && t.seconds() < SETTLE_S + MEASURE_S) {
                if (Math.abs(angle()) > LIMIT_RAD) {   // safety first
                    hitLimit[i] = true;
                    break;
                }
                if (t.seconds() > SETTLE_S) {          // only average after settling
                    sum += velocity();
                    n++;
                }
                telemetry.addData("Run", "%d / %d  power %.2f", i + 1, POWERS.length, power);
                telemetry.addData("Angle (deg)", "%.1f", Math.toDegrees(angle()));
                telemetry.addData("Vel (rad/s)", "%.2f", velocity());
                telemetry.update();
            }

            setPower(0);
            results[i] = (n > 0) ? sum / n : Double.NaN;
            sleep(500); // let it stop before reversing
        }

        setPower(0);

        // ---------- 4. SHOW RESULTS: keep them on screen until STOP ----------
        while (opModeIsActive()) {
            for (int i = 0; i < POWERS.length; i++) {
                telemetry.addData(String.format("power %+.2f", POWERS[i]),
                        hitLimit[i] ? "HIT LIMIT (shorten SETTLE/MEASURE or lower power)"
                                : String.format("%.3f rad/s", results[i]));
            }
            telemetry.update();
        }
    }

    double angle()    { return encoder.getCurrentPosition() / TICKS_PER_TURRET_RAD; }
    double velocity() { return encoder.getVelocity() / TICKS_PER_TURRET_RAD; }
    void setPower(double p) { s1.setPower(p); s2.setPower(p); }
}