package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
// @Configurable
//public class IntakeSensor extends Subsystem {
    // ---- ball detection (live in Panels) ----
//    public static double BALL_MM = 80;          // closer than this = ball there
//    public static double BLOCK_S = 0.5;         // ball must stay this long before the intake stops
    // ---- shootable zone: real distance to the goal (turret.dReal), inches ----
    // -1 = use the shot table's first / last row, so "shootable" = "the table covers this distance"
//    public static double ZONE_MIN_IN = -1;
//    public static double ZONE_MAX_IN = -1;
    // ---- LEDs ----
    // pin voltage that LIGHTS the LED: true = HIGH (pin -> resistor -> LED -> GND), false = LOW (3.3V -> LED -> pin)
    // public static boolean LED_ON_STATE = true;
//    public static long READ_PERIOD_MS = 40;     // pause between sensor reads so the thread doesn't hog the hub

//    private DistanceSensor sensor;
//    private Object ledBall, ledZone;   // DigitalChannel or LED, whichever the config uses
//    private volatile double distMm = Double.POSITIVE_INFINITY;   // latest reading from the thread
//    private volatile double readMs = 0;                          // how long one read takes (telemetry)
//    private volatile boolean running = false;
//    private Thread reader;
//    public boolean ballSeen = false;   // closer than BALL_MM right now
//    public boolean blocked = false;    // ball seen for longer than BLOCK_S -> stop intake
//    public boolean shootable = false;private long closeSinceNanos = -1;
//    private boolean lastBall = false, lastZone = false;

//    @Override
//    public void init(OpMode opMode) {
//        instantiateSubsystem(opMode);
//        sensor = hardwareMap.tryGet(DistanceSensor.class, "intake_dist");
//        ledBall = findLed("led_ball");
//        ledZone = findLed("led_zone");
//        setLed(ledBall, false);
//        setLed(ledZone, false);
//
//       if (sensor != null) {
//            running = true;
//            reader = new Thread(() -> {
//                while (running && !Thread.currentThread().isInterrupted()) {
//                    try {
//                        long t0 = System.nanoTime();
//                        double d = sensor.getDistance(DistanceUnit.MM);
//                        readMs = (System.nanoTime() - t0) / 1e6;
//                        distMm = Double.isNaN(d) ? Double.POSITIVE_INFINITY : d;
//                        Thread.sleep(READ_PERIOD_MS);   // a non-stop read loop floods the hub and slows the main loop
//                    } catch (Exception e) {
//                        break;   // OpMode ended / sensor gone
//                    }
//                }
//            }, "IntakeSensor");
//            reader.setDaemon(true);
//            reader.start();
//        }
//    }
//
//    /** Call once per loop, AFTER turret.tUpdate() (needs this loop's distance). */
//    public void update(turret t) {
//        // ---- ball ----
//        double d = distMm;
//        ballSeen = d < BALL_MM;
//        long now = System.nanoTime();
//       if (ballSeen) {
//            if (closeSinceNanos < 0) closeSinceNanos = now;
//            blocked = (now - closeSinceNanos) / 1e9 > BLOCK_S;
//        } else {
//            closeSinceNanos = -1;
//            blocked = false;           // nothing there -> intake runs normally
//        }

        // ---- shootable zone ----
//        double lo = ZONE_MIN_IN >= 0 ? ZONE_MIN_IN : ShotTable.D[0];
//        double hi = ZONE_MAX_IN >= 0 ? ZONE_MAX_IN : ShotTable.D[ShotTable.D.length - 1];
//        shootable = t.dReal >= lo && t.dReal <= hi && t.reachable && !t.faulted;

//        if (ballSeen != lastBall) { setLed(ledBall, ballSeen); lastBall = ballSeen; }   // each write = a hub command
//        if (shootable != lastZone) { setLed(ledZone, shootable); lastZone = shootable; }

//        telemetry.addData("intake sensor", sensor == null ? "NOT IN CONFIG (intake_dist)"
//                : String.format("%.0f mm  ball %s  %s  (read %.0f ms)", d > 8000 ? -1 : d,
//                ballSeen ? "YES" : "no", blocked ? "INTAKE STOPPED" : "", readMs));
//        telemetry.addData("shootable", "%s  (%.0f in, zone %.0f-%.0f)%s", shootable ? "YES" : "no",
//                t.dReal, lo, hi, (ledBall == null || ledZone == null) ? "  [LED missing in config]" : "");
//    }

//    /** Call from the OpMode's stop(). Ends the reader thread and turns the LEDs off. */
//    public void stop() {
//        running = false;
//        if (reader != null) reader.interrupt();
//        setLed(ledBall, false);
//        setLed(ledZone, false);
//    }

//    private Object findLed(String name) {
//        DigitalChannel c = hardwareMap.tryGet(DigitalChannel.class, name);
//        if (c != null) {
//            c.setMode(DigitalChannel.Mode.OUTPUT);
//            return c;
//        }
//        return hardwareMap.tryGet(LED.class, name);   // null if not in the config at all
//    }

//    private void setLed(Object led, boolean on) {
//        boolean high = (on == LED_ON_STATE);           // the pin voltage we want
//        if (led instanceof DigitalChannel) ((DigitalChannel) led).setState(high);
//        else if (led instanceof LED) ((LED) led).enable(!high);   // SDK LED type: enable(true) = pin LOW
//    }
//}
@Configurable
public class IntakeSensor {
    public static double BALL_THRESHOLD = 80.0; // closer then this == ball there
    public static double BALL_TIMER = 0.5; // amount ball must stay before can stop intake
    public static double ZONE_MIN_INCHES = -1; // gets inputs from shot tables until we input values
    public static double ZONE_MAX_INCHES = -1; // what i get from this, it means that if the distance is in beetween these two numbers (-1 is a placeholder) then it is in a ready position to shoot
    public static boolean LED_WIRING = true; // idk what claude wants from this tbh, some leds are wired so high = off, and low = on, while some are the oppisite, so you just true/false this if needed
    public static long WAIT_TIME_MS = 40; // threads run constantly, this is some wait time, so its not hogging the control hub and slowing down main code
    private DistanceSensor distanceSensor;
    private Object ledBall,ledShooting; // LED's can be part of either LED or Digital channel, object tracks both
    private volatile double latestDistanceReading = Double.POSITIVE_INFINITY; // can be any number, volatile forces the main loop to always read the newest data
    private volatile double timeToRead = 0; // how long the sensor takes to read the info, if its too high, the sensor is being detrimental to stuff
    private volatile boolean running = false; // this is volatile cuz threads need it to be for some reason
    private Thread reader; // makes a thread to constantly read the data
    public boolean ballSeen = false;
    public boolean blocked = false;
    public boolean shootable = false;
    private long closeSinceNanos = -1; // checks when the sensor started timing it
    private boolean lastBall = false, lastZone = false; // checks if last reading of the led, if the new time has to change the led's it does, otherwise it does nothing


    public void init(OpMode opMode) {
        hardwareMap.get(DistanceSensor.class,"Distance_Sensor"); // i dont want to do tryGet, seems kinda unnesesary
        ledBall = findLed("led_ball");
        ledShooting = findLed("led_shooting");
        setLed(ledBall,false);
        setLed(ledShooting,false); // turns them off

        if (distanceSensor != null) {
            running = true; // the sensor runs
            reader = new Thread(() -> { // this is a lambada; this line can also be written like this:    reader = new Thread(new Runnable() { @Override   public void run() {  }  });
                while (running && !Thread.currentThread().isInterrupted()) {
                    try { // use a try because sleep throws an interruption without try also i cut out unnesesary code
                        double distance = distanceSensor.getDistance(DistanceUnit.MM); // makes a variable that stores the distance in mm
                        double updatedDistance = Double.isNaN(distance) ? Double.POSITIVE_INFINITY : distance; // if distance doesnt exist, it stores it as a infinite, otherwise it takes the actual distance put in
                        Thread.sleep(WAIT_TIME_MS); // apperantly an always running loop slows down stuff so its good for it to rest
                    } catch (Exception blah) {
                        break; // if anything goes wrong in try, exeption catches it and breaks the loop
                    }
                }
            });
            reader.setDaemon(true); // marks it as backround thread, when everything else ends, it ends too
            reader.start(); // starts it yay
        }
    }
    public void update(turret t) {
        double newDistance = latestDistanceReading; // takes one snippet of the constantly changing variable, so theres no changes to it per line, it only resets at start of loop
        ballSeen = newDistance < BALL_THRESHOLD; // boolean so if the new distance is smaller then 80 it returns true
        long now = System.nanoTime(); // takes the time NOW
        if (ballSeen) {
            if (closeSinceNanos < 0) {
                closeSinceNanos = now;
            } // this confuses me, im guessing that if there is no recorded time of ball arrival, then it markes it down with now, and then jst skips the rest of it
            blocked = (now - closeSinceNanos) / 1e9 > BALL_TIMER; // ok so, now is a timer that updates per loop, closest gets stored once and does not change until reset, divide by 1e9 because we need it to be in seconds.
//               if the result of this math is greater then 0.5 seconds, then it puts in true
        } else {
            closeSinceNanos = -1;
            blocked = false;
            // keeps running intake, sets closeSinceNanos back to 0 basically so the if statement can run in the next loop
        }
        // this is one way to write this
        double low;
        if (ZONE_MIN_INCHES >= 0) {
            low = ZONE_MIN_INCHES; // if the number already exists, theres no need to change it
        } else {
            low = ShotTable.D[0]; // no clue what this is, but im guessing its an array with inputted values already
        }
        double high = ZONE_MAX_INCHES >= 0 ? ZONE_MIN_INCHES : ShotTable.D[ShotTable.D.length - 1];
        // this is another way of writing it, the reason its ShotTable.D.length - 1 is because there is no 6 position since it starts at 0
        shootable = t.dReal >= low && t.dReal <= high && t.reachable && !t.faulted; // if all of these are true: number is higher then lowest and lower then highest, turret can aim there and it doesnt have error, shootable = true

        if (ballSeen != lastBall) {
            setLed(ledBall, ballSeen);
            lastBall = ballSeen;
        }
        // ballSeen = what it should be. lastBall = what the led is displaying now, sets led to what ball seen is because thats what it needs to be; else last ball is ball seen, and updates to what it is, this gets changed earlier in loop
        if (shootable != lastZone) {
            setLed(ledShooting, shootable);
            lastZone = shootable;
        }
        // Shootable is what is should be. last Zone is what the led is displaying, sets led to what shootable is because that is was it needs to be, else last zone is shootable, and updates to waht it is, this gets changed earlier in the loop.

        telemetry.addData("distance", newDistance);
        telemetry.addData("blocked?", blocked);
        telemetry.addData("shootable?", shootable);
        // i shortened a crap ton of the telemetry, i think i udnerstand what it means, but its complicated for no reason, also i dont know if we need all these, or if thers more to include

        // i dont think we need stop, but if we do then we can do smth like this maybe
//  if(gamepad2.xWasPressed())  {
//      running = false;
//      setLed(ledShooting, false);
//      setLed(ledBall, false);
//      if (reader != null) { reader.interrupt(); }

    }

    private Object findLed(String random) { // this is a method that u give it an LED's name, and it gives you back the LED so your code can turn it on and off.
        DigitalChannel channel = hardwareMap.get(DigitalChannel.class, random); // makes a thing called channel
        if (channel != null) { // if the thing exists
            channel.setMode(DigitalChannel.Mode.OUTPUT); // turns it on
            return channel; // returns whatever it finds, not entirely shot
        } else {
            return hardwareMap.tryGet(DigitalChannel.class, random); // if it doesnt exist the try get returns null instead of crashing ur code
        }
    }

    private void setLed(Object led, boolean on) {
        boolean active = (on == LED_WIRING); // turns an LED on or off, no matter how it's wired,
        if (led instanceof DigitalChannel) { // if its a digital channel
            DigitalChannel newLed = (DigitalChannel) led; // sets the object of led as a digital channel led
            newLed.setState(active); // sets it to acitve
        } else if (led instanceof LED) { // if its part of the led
            LED otherLed = (LED) led; // sets it to be an LED instead of an object
            otherLed.enable(!active); // sets it to active
        }
    }
}


