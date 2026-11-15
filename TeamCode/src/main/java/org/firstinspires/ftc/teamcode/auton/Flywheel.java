package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

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
import org.firstinspires.ftc.teamcode.subSystems.Shooter;

@Configurable
public class Flywheel {

    // ---------------- flywheel ----------------
    public static boolean FLYWHEEL_REVERSE = true;   // flip if "rpm" reads negative while shooting (applied at init)
    public static double TARGET_RPM = 2090;
    public static double TICKS_PER_REV = 28;         // motor encoder ticks per rev
    // feedforward, fit with Flywheel Tuner: power needed at 12 V = kS + kV * (ticks/s)
    // NOTE: kS/kV/kP, bang thresholds and VEL_FILTER below are NOT USED any more:
    // updateFlywheel() reads Shooter's tuned values. Only TARGET_RPM / HOOD_POS here matter for auton.
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
    public static double HOOD_POS = 0.11;             // later: set from the shot table with setHood()

    // ---------------- intake ----------------
    public static double INTAKE_POWER = -1.0;
    public static double INTAKE_REVERSE_POWER = 0.4;

    // ---------------- transfer + gate ----------------
    public static double TRANSFER_FEED = -1.0;       // while shooting
    public static double TRANSFER_HOLD = -0.75;     // pushing balls against the closed gate: LOWER = less current
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
    public Servo gateServo, hoodServo, flowerServo;
    private VoltageSensor battery;

    // ---------------- state ----------------
    public boolean flywheelOn = false, intakeOn = false, transferOn = false, flowerOn = false;
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
        flowerServo = hardwareMap.get(Servo.class, "flower_servo");

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

        // hood ALWAYS (so it's already in place when the first ball arrives)
        hoodServo.setPosition(HOOD_POS);

        // Gate & Transfer control
        if (feeding) {
            gateServo.setPosition(GATE_OPEN);
            transferMotor.setPower(TRANSFER_FEED);
            if (rpm < burstMinRpm) {
                burstMinRpm = rpm;
            }
        } else {
            gateServo.setPosition(GATE_CLOSED);
            transferMotor.setPower(transferOn ? TRANSFER_HOLD : 0);
        }
        if(flowerOn) {flowerServo.setPosition(0);}
        else {flowerServo.setPosition(0.34);}
    }

    public void updateFlywheel() {
        if (voltageTimer.seconds() > VOLTAGE_PERIOD_S) {
            voltage = 0.7 * voltage + 0.3 * battery.getVoltage();
            voltageTimer.reset();
        }

        // Gains/filter come from Shooter (the TUNED teleop values) so auton and teleop always match.
        double velFilter = Shooter.VEL_FILTER;
        tps = velFilter * flywheel.getVelocity() + (1 - velFilter) * tps;
        rpm = tps / TICKS_PER_REV * 60.0;

        if (!flywheelOn) {
            flyPower = 0;
            bangMode = false;
            ready = false;
            flywheel.setPower(0);
            return;
        }

        double errRpm = TARGET_RPM - rpm;

        double enter = feeding ? Shooter.FEED_BANG_ENTER_RPM : Shooter.BANG_ENTER_RPM;
        double exit  = feeding ? Shooter.FEED_BANG_EXIT_RPM  : Shooter.BANG_EXIT_RPM;
        if (errRpm > enter) bangMode = true;
        else if (errRpm < exit) bangMode = false;

        double cmd;
        if (bangMode) {
            cmd = BANG_POWER;
        } else {
            double targetTps = TARGET_RPM / 60.0 * TICKS_PER_REV;
            double comp = 12.0 / Math.max(voltage, 8.0);
            cmd = (Shooter.kS_FLY + Shooter.kV_FLY * targetTps + Shooter.kP_FLY * (targetTps - tps)) * comp;
        }
        flyPower = Math.max(0, Math.min(1, cmd));
        flywheel.setPower(flyPower);

        ready = Math.abs(errRpm) < READY_RPM_TOL;
    }

    // ---------------- Ivy Commands ----------------
    // EXPLICIT on/off commands: unlike toggles they can't get out of sync if a step is added/removed.
    public Command flywheelOn()  { return instant(() -> flywheelOn = true); }
    public Command flywheelOff() { return instant(() -> flywheelOn = false); }
    public Command intakeOn()    { return instant(() -> intakeOn = true); }
    public Command intakeOff()   { return instant(() -> intakeOn = false); }
    public Command transferOn()  { return instant(() -> transferOn = true); }
    public Command transferOff() { return instant(() -> transferOn = false); }
    public Command flowerOn()    { return instant(() -> flowerOn = true); }
    public Command flowerOff()   { return instant(() -> flowerOn = false); }
    public Command allOff() {
        return instant(() -> { flywheelOn = false; intakeOn = false; transferOn = false; feeding = false; flowerOn = false; });
    }
    /** Set the shot for the NEXT shoot() (rpm + hood). */
    public Command setShot(double rpm, double hood) {
        return instant(() -> { TARGET_RPM = rpm; HOOD_POS = hood; });
    }
    /**
     * Shoot: wait until the flywheel is READY (max timeoutMs, so a slow flywheel can't freeze the auton),
     * settle a moment, open the gate + feed for feedMs, then close the gate.
     */
    public Command shoot(double feedMs, double timeoutMs) {
        return sequential(
                instant(() -> flywheelOn = true),
                race(waitUntil(() -> ready), waitMs(timeoutMs)),
                waitMs(250),                                   // let rpm settle after reaching the band
                instant(() -> { feeding = true; transferOn = true; burstMinRpm = rpm; }),
                waitMs(feedMs),
                instant(() -> { feeding = false; lastBurstMinRpm = burstMinRpm; })
        );
    }


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

    public Command toggleFlower(){
        return instant(() -> flowerOn = !flowerOn);
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

