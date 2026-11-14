package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.Shooter;

/**
 * Flywheel only (intake / transfer / gate never run).
 *
 * A = CHARACTERIZE: runs open-loop powers, measures steady speed + battery voltage,
 *     and fits power(at 12 V) = kS + kV * ticksPerSec. Copy kS_FLY / kV_FLY into Shooter.java.
 * Then TUNE (uses the real Shooter controller, gains live in Panels -> Shooter):
 *     X       flywheel on/off
 *     B       step target between LOW_RPM and HIGH_RPM   (tests spin-up AND slowing down)
 *     dpad up/down  target +/- 250 rpm
 * Graph in Panels: target, rpm, error, power, mode (1 = bang-bang, 0 = FF+P).
 */
@Configurable
@TeleOp(name = "Flywheel Tuner", group = "Test")
public class FlywheelTuner extends LinearOpMode {
    public static double LOW_RPM = 2500, HIGH_RPM = 3500;
    // Only measure AROUND the shooting speeds (~1900-2600 rpm): a straight line fit over 0.3-1.0
    // is a compromise that's off at both 2100 and 2400. Narrow range = accurate where you shoot.
    public static double[] CHAR_POWERS = {0.48, 0.53, 0.58, 0.63, 0.68, 0.73};
    public static double SETTLE_S = 3.0;    // flywheels are slow to settle
    public static double MEASURE_S = 1.0;

    Shooter shooter = new Shooter();
    PanelsPads pads = new PanelsPads();   // real + Panels controllers merged
    TelemetryManager panels;

    @Override
    public void runOpMode() {
        shooter.init(this);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Shooter.TARGET_RPM = LOW_RPM;

        telemetry.addLine("A = characterize (fit kS/kV).  X = on/off, B = step low/high, dpad = +/-250");
        telemetry.update();
        waitForStart();

        boolean lastA = false, lastB = false, lastX = false, lastUp = false, lastDown = false;
        String fitResult = "not run yet (press A)";

        while (opModeIsActive()) {
            pads.update(this);
            if (pads.g1.a && !lastA) fitResult = characterize();
            if (pads.g1.x && !lastX) shooter.flywheelOn = !shooter.flywheelOn;
            if (pads.g1.b && !lastB) Shooter.TARGET_RPM = (Shooter.TARGET_RPM == LOW_RPM) ? HIGH_RPM : LOW_RPM;
            if (pads.g1.dpad_up && !lastUp) Shooter.TARGET_RPM += 250;
            if (pads.g1.dpad_down && !lastDown) Shooter.TARGET_RPM = Math.max(0, Shooter.TARGET_RPM - 250);
            lastA = pads.g1.a; lastB = pads.g1.b; lastX = pads.g1.x;
            lastUp = pads.g1.dpad_up; lastDown = pads.g1.dpad_down;

            shooter.updateFlywheel();

            panels.addData("target rpm", Shooter.TARGET_RPM);
            panels.addData("rpm", shooter.rpm);
            panels.addData("error rpm", Shooter.TARGET_RPM - shooter.rpm);
            panels.addData("power", shooter.flyPower);
            panels.addData("mode (1=bang)", shooter.bangMode ? 1 : 0);
            panels.addData("battery V", shooter.voltage);
            panels.addData("READY", shooter.ready);
            panels.addData("on", shooter.flywheelOn);
            panels.addData("fit", fitResult);
            panels.update(telemetry);
        }
        shooter.flywheel.setPower(0);
    }

    /** Open-loop sweep; least-squares fit of voltage-normalized power vs speed. */
    private String characterize() {
        shooter.flywheelOn = false;
        int n = CHAR_POWERS.length;
        double[] x = new double[n], y = new double[n];   // x = ticks/s, y = power scaled to 12 V
        int got = 0;
        StringBuilder points = new StringBuilder();

        for (int i = 0; i < n && opModeIsActive(); i++) {
            double p = CHAR_POWERS[i];
            shooter.flywheel.setPower(p);
            ElapsedTime t = new ElapsedTime();
            double sumTps = 0, sumV = 0;
            int samples = 0;
            while (opModeIsActive() && t.seconds() < SETTLE_S + MEASURE_S) {
                if (t.seconds() > SETTLE_S) {
                    sumTps += shooter.flywheel.getVelocity();
                    sumV += shooter.readBatteryVoltage();
                    samples++;
                }
                telemetry.addData("Characterize", "power %.2f (%d/%d)  rpm %.0f", p, i + 1, n,
                        shooter.flywheel.getVelocity() / Shooter.TICKS_PER_REV * 60);
                telemetry.update();
            }
            if (samples > 0) {
                x[got] = sumTps / samples;
                y[got] = p * (sumV / samples) / 12.0;   // what this power would be at 12 V
                got++;
            }
            if (samples > 0) points.append(String.format("%.2f->%.0f  ", p, x[got - 1] / Shooter.TICKS_PER_REV * 60));
        }
        shooter.flywheel.setPower(0);
        if (got < 2) return "not enough data";

        double mx = 0, my = 0;
        for (int i = 0; i < got; i++) { mx += x[i]; my += y[i]; }
        mx /= got; my /= got;
        double sxy = 0, sxx = 0;
        for (int i = 0; i < got; i++) { sxy += (x[i] - mx) * (y[i] - my); sxx += (x[i] - mx) * (x[i] - mx); }
        double kV = sxy / sxx, kS = my - kV * mx;

        Shooter.kV_FLY = kV;   // applied live; copy into Shooter.java to keep
        Shooter.kS_FLY = kS;
        return String.format("kS_FLY = %.4f   kV_FLY = %.7f   (rpm range %.0f-%.0f)  points: %s", kS, kV,
                x[0] / Shooter.TICKS_PER_REV * 60, x[got - 1] / Shooter.TICKS_PER_REV * 60, points);
    }
}
