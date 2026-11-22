package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import robotcore.Subsystem;

/**
 * REV 2m distance sensor at the intake + 2 digital LEDs.
 *
 * Robot config names (Expansion Hub):
 *   "intake_dist"  REV 2m Distance Sensor, any I2C port
 *   "led_ball"     Digital Device, digital port 0-1, channel 0   -> ON while a ball is seen
 *   "led_zone"     Digital Device, digital port 0-1, channel 1   -> ON while the robot is in a shootable spot
 * The LEDs can be configured as "Digital Device" OR "LED" - both work.
 * Anything missing from the config is just skipped (no crash), telemetry says so.
 *
 * Intake: a ball closer than BALL_MM for longer than BLOCK_S -> `blocked` = true -> Shooter stops the intake.
 * Nothing seen -> not blocked -> intake runs normally. Reverse (right trigger) always works, to clear jams.
 *
 * The 2m sensor is SLOW (a read can take ~20-30 ms), so it is read in a background thread
 * and never stalls the main loop (turret + flywheel control stay fast).
 */
@Configurable
public class IntakeSensor extends Subsystem {
    // ---- ball detection (live in Panels) ----
    public static double BALL_MM = 80;          // closer than this = ball there
    public static double BLOCK_S = 0.5;         // ball must stay this long before the intake stops
    // ---- shootable zone: real distance to the goal (turret.dReal), inches ----
    // -1 = use the shot table's first / last row, so "shootable" = "the table covers this distance"
    public static double ZONE_MIN_IN = -1;
    public static double ZONE_MAX_IN = -1;
    // ---- LEDs ----
    // pin voltage that LIGHTS the LED: true = HIGH (pin -> resistor -> LED -> GND), false = LOW (3.3V -> LED -> pin)
    public static boolean LED_ON_STATE = true;
    public static long READ_PERIOD_MS = 100;    // pause between sensor reads so the thread doesn't hog the hub

    private DistanceSensor sensor;
    private Object ledBall, ledZone;   // DigitalChannel or LED, whichever the config uses

    private volatile double distMm = Double.POSITIVE_INFINITY;   // latest reading from the thread
    private volatile double readMs = 0;                          // how long one read takes (telemetry)
    private volatile boolean running = false;
    private Thread reader;

    public boolean ballSeen = false;   // closer than BALL_MM right now
    public boolean blocked = false;    // ball seen for longer than BLOCK_S -> stop intake
    public boolean shootable = false;
    private long closeSinceNanos = -1;
    private boolean lastBall = false, lastZone = false;

    @Override
    public void init(OpMode opMode) {
        instantiateSubsystem(opMode);
        sensor = hardwareMap.tryGet(DistanceSensor.class, "intake_dist");
        ledBall = findLed("led_ball");
        ledZone = findLed("led_zone");
        setLed(ledBall, false);
        setLed(ledZone, false);

        if (sensor != null) {
            running = true;
            reader = new Thread(() -> {
                while (running && !Thread.currentThread().isInterrupted()) {
                    try {
                        long t0 = System.nanoTime();
                        double d = sensor.getDistance(DistanceUnit.MM);
                        readMs = (System.nanoTime() - t0) / 1e6;
                        distMm = Double.isNaN(d) ? Double.POSITIVE_INFINITY : d;
                        Thread.sleep(READ_PERIOD_MS);   // a non-stop read loop floods the hub and slows the main loop
                    } catch (Exception e) {
                        break;   // OpMode ended / sensor gone
                    }
                }
            }, "IntakeSensor");
            reader.setDaemon(true);
            reader.start();
        }
    }

    /** Call once per loop, AFTER turret.tUpdate() (needs this loop's distance). */
    public void update(turret t) {
        // ---- ball ----
        double d = distMm;
        ballSeen = d < BALL_MM;
        long now = System.nanoTime();
        if (ballSeen) {
            if (closeSinceNanos < 0) closeSinceNanos = now;
            blocked = (now - closeSinceNanos) / 1e9 > BLOCK_S;
        } else {
            closeSinceNanos = -1;
            blocked = false;           // nothing there -> intake runs normally
        }

        // ---- shootable zone ----
        double lo = ZONE_MIN_IN >= 0 ? ZONE_MIN_IN : ShotTable.D[0];
        double hi = ZONE_MAX_IN >= 0 ? ZONE_MAX_IN : ShotTable.D[ShotTable.D.length - 1];
        shootable = t.dReal >= lo && t.dReal <= hi && t.reachable && !t.faulted;

        if (ballSeen != lastBall) { setLed(ledBall, ballSeen); lastBall = ballSeen; }   // each write = a hub command
        if (shootable != lastZone) { setLed(ledZone, shootable); lastZone = shootable; }

        if (!showTelemetry) return;
        telemetry.addData("intake sensor", sensor == null ? "NOT IN CONFIG (intake_dist)"
                : String.format("%.0f mm  ball %s  %s  (read %.0f ms)", d > 8000 ? -1 : d,
                ballSeen ? "YES" : "no", blocked ? "INTAKE STOPPED" : "", readMs));
        telemetry.addData("shootable", "%s  (%.0f in, zone %.0f-%.0f)%s", shootable ? "YES" : "no",
                t.dReal, lo, hi, (ledBall == null || ledZone == null) ? "  [LED missing in config]" : "");
    }

    /** Call from the OpMode's stop(). Ends the reader thread and turns the LEDs off. */
    public void stop() {
        running = false;
        if (reader != null) reader.interrupt();
        setLed(ledBall, false);
        setLed(ledZone, false);
    }

    private Object findLed(String name) {
        DigitalChannel c = hardwareMap.tryGet(DigitalChannel.class, name);
        if (c != null) {
            c.setMode(DigitalChannel.Mode.OUTPUT);
            return c;
        }
        return hardwareMap.tryGet(LED.class, name);   // null if not in the config at all
    }

    private void setLed(Object led, boolean on) {
        boolean high = (on == LED_ON_STATE);           // the pin voltage we want
        if (led instanceof DigitalChannel) ((DigitalChannel) led).setState(high);
        else if (led instanceof LED) ((LED) led).enable(!high);   // SDK LED type: enable(true) = pin LOW
    }
}
