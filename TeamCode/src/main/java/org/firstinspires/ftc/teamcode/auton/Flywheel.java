package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class Flywheel {

    // ---------------- flywheel ----------------
    public static boolean FLYWHEEL_REVERSE = true;   // flip if "rpm" reads negative while shooting (applied at init)
    public static double TARGET_RPM = 2100;
    public static double TICKS_PER_REV = 28;         // motor encoder ticks per rev
    // feedforward, fit with Flywheel Tuner: power needed at 12 V = kS + kV * (ticks/s)
    public static double kS_FLY = 0.028;
    public static double kV_FLY = 0.0004719;      // placeholder (MotorOuttake's value) until you fit it
    public static double kP_FLY = 0.0018;             // power per (tick/s) of speed error
    // bang-bang
    public static double BANG_POWER = 1.0;
    public static double BANG_ENTER_RPM = 210;       // go full power when this far BELOW target
    public static double BANG_EXIT_RPM = 200;        // hand back to FF+P when within this much below target
    // while LB is held (balls going through), recover HARDER: full power as soon as we're a little slow
    public static double FEED_BANG_ENTER_RPM = 30;   // full power when this far below target WHILE FEEDING
    public static double FEED_BANG_EXIT_RPM = 5;     // back to FF+P when within this much
    public static double READY_RPM_TOL = 75;         // "ready to shoot" band
    public static double VEL_FILTER = 0.01;           // 1 = raw velocity, smaller = smoother but laggier
    public static double VOLTAGE_PERIOD_S = 0.25;    // how often to read battery voltage

    // ---------------- hood ----------------
    public static double HOOD_POS = 0.09;             // later: set from the shot table with setHood()

    // ---------------- intake ----------------
    public static double INTAKE_POWER = -0.6;
    public static double INTAKE_REVERSE_POWER = 0.4;

    // ---------------- transfer + gate ----------------
    public static double TRANSFER_FEED = -1.0;       // while shooting
    public static double TRANSFER_HOLD = -0.675;     // pushing balls against the closed gate: LOWER = less current
    public static double GATE_CLOSED = 0.4;          // swap these two if the gate works backwards
    public static double GATE_OPEN = 0.65;
    // Auto shot: MainOpMode sets RPM + hood from ShotTable every loop (using turret.d).
    // Leave false until ShotTable has REAL numbers in it.
    public static boolean AUTO_SHOT = true;

    public static boolean COMP_INTAKE_TRANSFER = true; // scale intake/transfer power by 12 V / battery so they don't slow when it sags
    public static boolean SERVOS_ON = true;         // TEST SWITCH: false = hood + gate servos go limp (PWM off)
    public static boolean SHOW_CURRENTS = false;     // each current read costs loop time; turn off for matches

    // ---------------- hardware ----------------
    public DcMotorEx flywheel, intakeMotor, transferMotor;
    public Servo gateServo, hoodServo;
    private VoltageSensor battery;

    // ---------------- state ----------------
    public boolean flywheelOn = false, intakeOn = false, transferOn = false;
    public boolean intakeBlocked = false;   // set by IntakeSensor: ball sitting at the intake -> stop it
    public boolean ready = false, feeding = false, bangMode = false;
    public double rpm = 0, tps = 0, flyPower = 0, voltage = 12;
    public double hoodFix = 0;                            // hood change from the slow-flywheel fix
    public double burstMinRpm = 0, lastBurstMinRpm = 0;   // lowest rpm during the current / last LB burst
    private final ElapsedTime voltageTimer = new ElapsedTime();

    /**
     * Constructor for Pedro Pathing Ivy.
     * Pass hardwareMap (and optionally telemetry) when instantiating in your OpMode.
     */
    public Flywheel(HardwareMap hardwareMap) {

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
        hoodServo.setDirection(Servo.Direction.REVERSE);

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
        voltageTimer.reset();
    }

    /**
     * Call this once per loop in your OpMode (e.g., inside loop() right before/after Scheduler.run())
     * to keep flywheel velocity control, motor powers, and telemetry updated continuously.
     */
    public void periodic() {
        updateFlywheel();
        updateMechanisms();
    }

    private void updateMechanisms() {
        // Intake control
        if (intakeOn) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0);
        }

        // Gate & Transfer control
        if (feeding) {
            gateServo.setPosition(GATE_OPEN);
            transferMotor.setPower(TRANSFER_FEED);
            hoodServo.setPosition(HOOD_POS);
            if (rpm < burstMinRpm) {
                burstMinRpm = rpm;
            }
        } else {
            gateServo.setPosition(GATE_CLOSED);
            transferMotor.setPower(transferOn ? TRANSFER_HOLD : 0);
        }
    }

    public void updateFlywheel() {
        if (voltageTimer.seconds() > VOLTAGE_PERIOD_S) {
            voltage = 0.7 * voltage + 0.3 * battery.getVoltage();
            voltageTimer.reset();
        }

        tps = VEL_FILTER * flywheel.getVelocity() + (1 - VEL_FILTER) * tps;
        rpm = tps / TICKS_PER_REV * 60.0;

        if (!flywheelOn) {
            flyPower = 0;
            bangMode = false;
            ready = false;
            flywheel.setPower(0);
            return;
        }

        double errRpm = TARGET_RPM - rpm;

        double enter = feeding ? FEED_BANG_ENTER_RPM : BANG_ENTER_RPM;
        double exit  = feeding ? FEED_BANG_EXIT_RPM  : BANG_EXIT_RPM;
        if (errRpm > enter) bangMode = true;
        else if (errRpm < exit) bangMode = false;

        double cmd;
        if (bangMode) {
            cmd = BANG_POWER;
        } else {
            double targetTps = TARGET_RPM / 60.0 * TICKS_PER_REV;
            double comp = 12.0 / Math.max(voltage, 8.0);
            cmd = (kS_FLY + kV_FLY * targetTps + kP_FLY * (targetTps - tps)) * comp;
        }
        flyPower = Math.max(0, Math.min(1, cmd));
        flywheel.setPower(flyPower);

        ready = Math.abs(errRpm) < READY_RPM_TOL;
    }

    // ---------------- Ivy Commands ----------------

    /**
     * Toggles the flywheel on or off immediately.
     */
    public Command toggleFlywheel() {
        return instant(() -> flywheelOn = !flywheelOn);
    }

    /**
     * Turns the flywheel on and waits until it reaches TARGET_RPM (great for Auto).
     */
    public Command spinUpAndWait() {
        return Command.build()
                .setStart(() -> flywheelOn = true);
    }

    /**
     * Toggles the intake on or off.
     */
    public Command toggleIntake() {
        return instant(() -> intakeOn = !intakeOn);
    }

    /**
     * Reverses the intake while held (bind with whileTrue in Ivy).
     */

    /**
     * Toggles the transfer hold state on or off.
     */
    public Command toggleTransfer() {
        return instant(() -> transferOn = !transferOn);
    }

    public Command toggleFeeding(){
        return instant (() -> feeding = !feeding);
    }

    /**
     * Opens the gate and feeds into the flywheel while held (only if flywheel is on).
     */
    public Command shootWhileHeld() {
        return Command.build()
                .setStart(() -> {
                    if (flywheelOn) {
                        feeding = true;
                        burstMinRpm = rpm;
                    }
                })
                .setEnd(interrupted -> {
                    if (feeding) {
                        lastBurstMinRpm = burstMinRpm;
                        feeding = false;
                    }
                })
                .requiring(this);
    }

    /**
     * Feeds for a fixed duration in seconds (great for Auto sequences).
     */
    public Command shootForSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(() -> {
                    timer.reset();
                    flywheelOn = true;
                    feeding = true;
                    burstMinRpm = rpm;
                })
                .setEnd(interrupted -> {
                    lastBurstMinRpm = burstMinRpm;
                    feeding = false;
                })
                .requiring(this);
    }

    // ---------------- Getters & Telemetry ----------------

    public boolean isReady() {
        return ready;
    }

    public void setHoodPosition(double position){
        hoodServo.setPosition(position);
    }
}

