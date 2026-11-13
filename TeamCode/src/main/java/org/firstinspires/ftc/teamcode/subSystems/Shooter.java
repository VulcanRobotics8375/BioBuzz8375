package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import robotcore.Subsystem;

/**
 * Intake + transfer + gate + flywheel + hood.
 * REPLACES Intake and MotorOuttake in MainOpMode (don't run them together).
 *
 * Controls (gamepad2):
 *   Y            flywheel on/off
 *   X            intake on/off        (hold right trigger = reverse intake)
 *   A            transfer on/off      (when on and not shooting: low HOLD power against the gate)
 *   left bumper  HOLD to shoot: gate opens + transfer feeds everything (flywheel must be on)
 *
 * Flywheel = bang-bang + feedforward/P hybrid:
 *   far below target (> BANG_ENTER_RPM low)  -> BANG_POWER (full) to get up to speed fast
 *   near target, or above it                  -> power = (kS + kV*target + kP*error) * 12V/battery
 *   Hysteresis: bang-bang starts at BANG_ENTER_RPM below, hands over at BANG_EXIT_RPM below.
 *   Never reverse power: too fast -> FF+P lowers power and it coasts down.
 * Fit kS_FLY / kV_FLY with the "Flywheel Tuner" OpMode, then tune kP_FLY there.
 *
 * Tune live in Panels -> Configurables -> Shooter. Copy good values back into code.
 */
@Configurable
public class Shooter extends Subsystem {

    // ---------------- flywheel ----------------
    public static boolean FLYWHEEL_REVERSE = true;   // flip if "rpm" reads negative while shooting (applied at init)
    public static double TARGET_RPM = 2325;
    public static double TICKS_PER_REV = 28;         // motor encoder ticks per rev
    // feedforward, fit with Flywheel Tuner: power needed at 12 V = kS + kV * (ticks/s)
    public static double kS_FLY = 0.1288;
    public static double kV_FLY = 0.0004616;      // placeholder (MotorOuttake's value) until you fit it
    public static double kP_FLY = 0.03;             // power per (tick/s) of speed error
    // bang-bang
    public static double BANG_POWER = 1.0;
    public static double BANG_ENTER_RPM = 300;       // go full power when this far BELOW target
    public static double BANG_EXIT_RPM = 100;        // hand back to FF+P when within this much below target
    // while LB is held (balls going through), recover HARDER: full power as soon as we're a little slow
    public static double FEED_BANG_ENTER_RPM = 30;   // full power when this far below target WHILE FEEDING
    public static double FEED_BANG_EXIT_RPM = 5;     // back to FF+P when within this much
    public static double READY_RPM_TOL = 75;         // "ready to shoot" band
    public static double VEL_FILTER = 0.5;           // 1 = raw velocity, smaller = smoother but laggier
    public static double VOLTAGE_PERIOD_S = 0.25;    // how often to read battery voltage

    // ---------------- hood ----------------
    public static double HOOD_POS = 0.06;             // later: set from the shot table with setHood()

    // ---------------- intake ----------------
    public static double INTAKE_POWER = -0.6;
    public static double INTAKE_REVERSE_POWER = 0.4;

    // ---------------- transfer + gate ----------------
    public static double TRANSFER_FEED = -1.0;       // while shooting
    public static double TRANSFER_HOLD = -0.675;      // pushing balls against the closed gate: LOWER = less current
    public static double GATE_CLOSED = 0.4;          // swap these two if the gate works backwards
    public static double GATE_OPEN = 0.65;
    // Auto shot: MainOpMode sets RPM + hood from ShotTable every loop (using turret.d).
    // Leave false until ShotTable has REAL numbers in it.
    public static boolean AUTO_SHOT = false;

    public static boolean COMP_INTAKE_TRANSFER = true; // scale intake/transfer power by 12 V / battery so they don't slow when it sags
    public static boolean SERVOS_ON = true;         // TEST SWITCH: false = hood + gate servos go limp (PWM off)
    public static boolean SHOW_CURRENTS = false;     // each current read costs loop time; turn off for matches

    // ---------------- hardware ----------------
    public DcMotorEx flywheel, intakeMotor, transferMotor;
    public Servo gateServo, hoodServo;
    private VoltageSensor battery;

    // ---------------- state ----------------
    public boolean flywheelOn = false, intakeOn = false, transferOn = false;
    public boolean ready = false, feeding = false, bangMode = false;
    public double rpm = 0, tps = 0, flyPower = 0, voltage = 12;
    public double burstMinRpm = 0, lastBurstMinRpm = 0;   // lowest rpm during the current / last LB burst
    private final ElapsedTime voltageTimer = new ElapsedTime();

    @Override
    public void init(OpMode opMode) {
        instantiateSubsystem(opMode);

        flywheel = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        flywheel.setDirection(FLYWHEEL_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);          // we do our own speed control
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast, no braking spike

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gateServo = hardwareMap.get(Servo.class, "transfer_servo");
        hoodServo = hardwareMap.get(Servo.class, "hood_servo");

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
        voltageTimer.reset();
    }

    public void setTargetRPM(double targetRpm) { TARGET_RPM = targetRpm; }

    /** Distance (in, from turret.d) -> RPM + hood from the shot table. Only if AUTO_SHOT. */
    public void setShot(double distance) {
        if (!AUTO_SHOT) return;
        TARGET_RPM = ShotTable.rpm(distance);
        HOOD_POS = ShotTable.hood(distance, TARGET_RPM - rpm);   // + optional slow-flywheel hood fix
    }
    public void setHood(double pos) { HOOD_POS = pos; }
    public double readBatteryVoltage() { return battery.getVoltage(); }

    /** Call once per loop. Reads gamepad2 and runs everything. */
    public void update() {
        // ---- controls ----
        if (gamepad2.yWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad2.xWasPressed()) intakeOn = !intakeOn;
        if (gamepad2.aWasPressed()) transferOn = !transferOn;
        boolean shootHeld = gamepad2.left_bumper;

        boolean wasFeeding = feeding;
        feeding = shootHeld && flywheelOn;   // hold LB = everything fires, no waiting on READY
        if (feeding && !wasFeeding) burstMinRpm = rpm;          // burst started
        if (!feeding && wasFeeding) lastBurstMinRpm = burstMinRpm; // burst ended: remember the dip

        updateFlywheel();   // uses `feeding` for the harder recovery
        if (feeding) burstMinRpm = Math.min(burstMinRpm, rpm);
        // ---- hood + gate servos (SERVOS_ON = false turns them off completely, for power testing) ----
        if (SERVOS_ON) {
            hoodServo.setPosition(HOOD_POS);
            gateServo.setPosition(feeding ? GATE_OPEN : GATE_CLOSED);
        } else {
            if (hoodServo instanceof ServoImplEx) ((ServoImplEx) hoodServo).setPwmDisable();
            if (gateServo instanceof ServoImplEx) ((ServoImplEx) gateServo).setPwmDisable();
        }

        // ---- transfer ----
        if (feeding) transferMotor.setPower(vc(TRANSFER_FEED));
        else if (transferOn) transferMotor.setPower(vc(TRANSFER_HOLD));
        else transferMotor.setPower(0);

        // ---- intake ----
        if (!intakeOn) intakeMotor.setPower(0);
        else if (gamepad2.right_trigger_pressed) intakeMotor.setPower(vc(INTAKE_REVERSE_POWER));
        else intakeMotor.setPower(vc(INTAKE_POWER));

        addTelemetry();
    }

    /** Voltage-compensated power: same speed at 10.5 V as at 12 V (up to the +/-1 limit). */
    private double vc(double power) {
        if (!COMP_INTAKE_TRANSFER) return power;
        double p = power * 12.0 / Math.max(voltage, 8.0);
        return Math.max(-1, Math.min(1, p));
    }

    /** Flywheel controller only (no gamepad). The Flywheel Tuner calls this directly. */
    public void updateFlywheel() {
        // battery voltage: read slowly and smooth (noisy, and each read costs time)
        if (voltageTimer.seconds() > VOLTAGE_PERIOD_S) {
            voltage = 0.7 * voltage + 0.3 * battery.getVoltage();
            voltageTimer.reset();
        }

        tps = VEL_FILTER * flywheel.getVelocity() + (1 - VEL_FILTER) * tps;   // ticks per second, smoothed
        rpm = tps / TICKS_PER_REV * 60.0;

        if (!flywheelOn) {
            flyPower = 0;
            bangMode = false;
            ready = false;
            flywheel.setPower(0);
            return;
        }

        double errRpm = TARGET_RPM - rpm;   // + = too slow, - = too fast

        // hysteresis: in between EXIT and ENTER, keep whatever mode we were already in
        double enter = feeding ? FEED_BANG_ENTER_RPM : BANG_ENTER_RPM;
        double exit  = feeding ? FEED_BANG_EXIT_RPM  : BANG_EXIT_RPM;
        if (errRpm > enter) bangMode = true;
        else if (errRpm < exit) bangMode = false;

        double cmd;
        if (bangMode) {
            cmd = BANG_POWER;
        } else {
            double targetTps = TARGET_RPM / 60.0 * TICKS_PER_REV;
            double comp = 12.0 / Math.max(voltage, 8.0);           // sagging battery -> push harder
            cmd = (kS_FLY + kV_FLY * targetTps + kP_FLY * (targetTps - tps)) * comp;
        }
        flyPower = Math.max(0, Math.min(1, cmd));                  // never reverse: too fast = coast down
        flywheel.setPower(flyPower);

        ready = Math.abs(errRpm) < READY_RPM_TOL;
    }

    private void addTelemetry() {
        telemetry.addData("flywheel", "%s  target %.0f  rpm %.0f  power %.2f  %s",
                flywheelOn ? "ON" : "off", TARGET_RPM, rpm, flyPower, bangMode ? "BANG" : "FF+P");
        telemetry.addData("READY", ready);
        telemetry.addData("burst dip", "now %.0f  last burst %.0f rpm below target",
                feeding ? TARGET_RPM - burstMinRpm : 0, TARGET_RPM - lastBurstMinRpm);
        telemetry.addData("feeding", feeding);
        telemetry.addData("intake / transfer", "%s / %s",
                intakeOn ? "on" : "off", feeding ? "FEED" : (transferOn ? "HOLD" : "off"));
        telemetry.addData("battery V", "%.2f", voltage);
        if (SHOW_CURRENTS) {
            telemetry.addData("amps fly / intake / transfer", "%.1f / %.1f / %.1f",
                    flywheel.getCurrent(CurrentUnit.AMPS),
                    intakeMotor.getCurrent(CurrentUnit.AMPS),
                    transferMotor.getCurrent(CurrentUnit.AMPS));
        }
    }
}
